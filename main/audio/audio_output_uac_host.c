/*
 * USB Audio Class HOST output for ESP32-S3 and external USB DACs.
 * Original AirPlay software: Copyright (c) 2026 Remi Bouteiller.
 * This backend: experimental integration; preserve repository LICENSE.
 *
 * Supports USB Audio Class 1.0 playback interfaces offering 48 kHz,
 * 16-bit, 2-channel PCM. Unsupported devices are logged and ignored.
 * Never enable USB-device UAC at the same time as this host backend.
 */
#include "audio_output.h"
#include "audio_receiver.h"
#include "audio_resample.h"
#include "sf_peq_bridge.h" /* SF_PEQ_BRIDGE_INTEGRATED */
#include "led.h"
#include "rtsp_server.h"
#include "esp_log.h"
#include "usb/usb_host.h"
#include "usb/uac_host.h"
#include "freertos/task.h"
#include "freertos/FreeRTOS.h"
#include <stdlib.h>
#include <string.h>

#define TAG "d07_uac_host"
#define RATE 48000
#define FRAME 352
#define MAX_OUT 400
#define UAC_BUF_SIZE 16384
#define UAC_BUF_THRESHOLD 8192
#if CONFIG_FREERTOS_UNICORE
#define PLAY_CORE 0
#else
#define PLAY_CORE 1
#endif

static volatile bool s_pending = false;
static volatile bool s_unplugged = false;
static volatile bool s_flush = false;
static volatile bool s_running = true;
static volatile bool s_playback_started = false;
static volatile int s_source_rate = 44100;
static volatile bool s_new_rate = false;
static volatile uint8_t s_addr, s_iface;
static uac_host_device_handle_t s_dev;
static bool s_stream_started;

static void on_driver(uint8_t addr, uint8_t iface,
                      uac_host_driver_event_t event, void *arg) {
    (void)arg;
    if (event == UAC_HOST_DRIVER_EVENT_TX_CONNECTED) {
        ESP_LOGI(TAG, "UAC playback interface: address=%u iface=%u", addr, iface);
        s_addr = addr;
        s_iface = iface;
        s_pending = true;
    }
}
static void on_device(uac_host_device_handle_t handle,
                      uac_host_device_event_t event, void *arg) {
    (void)handle; (void)arg;
    if (event == UAC_HOST_DRIVER_EVENT_DISCONNECTED ||
        event == UAC_HOST_DEVICE_EVENT_TRANSFER_ERROR) {
        s_unplugged = true;
    }
}
static void host_events(void *arg) {
    (void)arg;
    while (true) {
        uint32_t flags = 0;
        esp_err_t err = usb_host_lib_handle_events(portMAX_DELAY, &flags);
        if (err != ESP_OK) ESP_LOGW(TAG, "Host event: %s", esp_err_to_name(err));
    }
}
static bool supports_pcm_48k(uac_host_device_handle_t dev) {
    uac_host_dev_info_t info = {0};
    if (uac_host_get_device_info(dev, &info) != ESP_OK) return false;
    ESP_LOGI(TAG, "USB DAC VID=%04x PID=%04x alternate-settings=%u",
             info.VID, info.PID, info.iface_alt_num);
    for (uint8_t alt = 1; alt <= info.iface_alt_num; ++alt) {
        uac_host_dev_alt_param_t p = {0};
        if (uac_host_get_device_alt_param(dev, alt, &p) != ESP_OK) continue;
        /* SF_D07_FORMAT_DIAGNOSTICS_V1: inspect modes without changing playback. */
        ESP_LOGI(TAG, "UAC alt=%u format=%u channels=%u bits=%u subframe=%u freq_count=%u",
                 (unsigned)alt, (unsigned)p.format, (unsigned)p.channels,
                 (unsigned)p.bit_resolution, (unsigned)p.subframe_size,
                 (unsigned)p.sample_freq_type);
        if (p.sample_freq_type == 0) {
            ESP_LOGI(TAG, "UAC alt=%u continuous rate=%lu..%lu Hz",
                     (unsigned)alt, (unsigned long)p.sample_freq_lower,
                     (unsigned long)p.sample_freq_upper);
        } else {
            for (size_t fi = 0; fi < p.sample_freq_type && fi < UAC_FREQ_NUM_MAX; ++fi) {
                ESP_LOGI(TAG, "UAC alt=%u rate[%u]=%lu Hz",
                         (unsigned)alt, (unsigned)fi, (unsigned long)p.sample_freq[fi]);
            }
        }
        if (p.format == 1 && p.channels == 2 && p.bit_resolution == 24 &&
            (p.subframe_size == 3 || p.subframe_size == 4)) {
            bool hz96 = (p.sample_freq_type == 0)
                ? (p.sample_freq_lower <= 96000 && p.sample_freq_upper >= 96000)
                : false;
            for (size_t fi = 0; !hz96 && fi < p.sample_freq_type && fi < UAC_FREQ_NUM_MAX; ++fi)
                hz96 = p.sample_freq[fi] == 96000;
            if (hz96) ESP_LOGW(TAG, "CANDIDATE 24-bit/96kHz stereo mode on alt=%u, %u-byte subframe (not yet activated)",
                                (unsigned)alt, (unsigned)p.subframe_size);
        }
        if (p.format != 1 || p.channels != 2 || p.bit_resolution != 16 || p.subframe_size != 2)
            continue;
        if (p.sample_freq_type == 0) {
            if (p.sample_freq_lower <= RATE && p.sample_freq_upper >= RATE) return true;
        } else {
            for (size_t i = 0; i < p.sample_freq_type && i < UAC_FREQ_NUM_MAX; ++i) {
                if (p.sample_freq[i] == RATE) return true;
            }
        }
    }
    return false;
}
static void close_dac(void) {
    if (s_dev) {
        if (s_stream_started) {
            (void)uac_host_device_stop(s_dev);
            s_stream_started = false;
        }
        (void)uac_host_device_close(s_dev);
        s_dev = NULL;
    }
    s_unplugged = false;
}
static void open_dac(void) {
    uint8_t addr = s_addr, iface = s_iface;
    s_pending = false;
    uac_host_device_config_t cfg = {
        .addr = addr, .iface_num = iface,
        .buffer_size = UAC_BUF_SIZE,
        .buffer_threshold = UAC_BUF_THRESHOLD,
        .callback = on_device,
        .callback_arg = NULL,
    };
    esp_err_t err = uac_host_device_open(&cfg, &s_dev);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "DAC open failed: %s", esp_err_to_name(err));
        s_dev = NULL;
        return;
    }
    if (!supports_pcm_48k(s_dev)) {
        ESP_LOGE(TAG, "DAC lacks UAC1 48kHz/16-bit/stereo PCM playback");
        close_dac();
        return;
    }
    uac_host_stream_config_t stream = {
        .channels = 2, .bit_resolution = 16, .sample_freq = RATE, .flags = 0,
    };
    err = uac_host_device_start(s_dev, &stream);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "DAC stream start failed: %s", esp_err_to_name(err));
        close_dac();
        return;
    }
    s_stream_started = true;
    ESP_LOGI(TAG, "DAC playback ready: 48000 Hz, 16-bit stereo");
}
static void apply_volume(int16_t *pcm, size_t samples) {
#ifndef CONFIG_DAC_CONTROLS_VOLUME
    static int32_t gain = -1;
    int32_t target = airplay_get_volume_q15();
    if (gain < 0) gain = target;
    for (size_t i = 0; i < samples; ++i) {
        if ((i & 1) == 0 && gain != target) {
            int32_t d = target - gain;
            int32_t step = d / 256;
            if (!step) step = (d > 0) ? 1 : -1;
            gain += step;
        }
        pcm[i] = (int16_t)(((int32_t)pcm[i] * gain) >> 15);
    }
#endif
}
static void playback(void *arg) {
    (void)arg;
    int16_t *pcm = malloc((FRAME + 1) * 2 * sizeof(int16_t));
    int16_t *out = malloc(MAX_OUT * 2 * sizeof(int16_t));
    if (!pcm || !out) {
        ESP_LOGE(TAG, "PCM buffer allocation failed");
        free(pcm); free(out); vTaskDelete(NULL); return;
    }
    while (s_running) {
        if (s_unplugged) {
            ESP_LOGW(TAG, "USB DAC disconnected; closing interface");
            close_dac();
        }
        if (!s_dev && s_pending) open_dac();
        if (s_new_rate) {
            s_new_rate = false;
            audio_resample_init((uint32_t)s_source_rate, RATE, 2);
        }
        if (s_flush) {
            s_flush = false;
            audio_resample_reset();
        }
        if (!s_stream_started) {
            // Keep draining AirPlay audio while no DAC is available, avoiding
            // stale bursts after it is reconnected.
            (void)audio_receiver_read(pcm, FRAME);
            vTaskDelay(pdMS_TO_TICKS(8));
            continue;
        }
        size_t frames = audio_receiver_read(pcm, FRAME + 1);
        if (!frames) {
            memset(pcm, 0, FRAME * 4);
            frames = FRAME;
        }
        int16_t *play = pcm;
        size_t output_frames = frames;
        if (audio_resample_is_active()) {
            output_frames = audio_resample_process(pcm, frames, out, MAX_OUT);
            play = out;
        }
        if (!output_frames) continue;
        sf_peq_bridge_process_i16(play, output_frames);
        apply_volume(play, output_frames * 2);
        led_audio_feed(play, output_frames);
        esp_err_t err = uac_host_device_write(s_dev, (uint8_t *)play,
                                              (uint32_t)(output_frames * 4), pdMS_TO_TICKS(30));
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "USB PCM write: %s", esp_err_to_name(err));
            if (err == ESP_ERR_INVALID_STATE || err == ESP_ERR_NOT_FOUND) s_unplugged = true;
            vTaskDelay(pdMS_TO_TICKS(2));
        }
    }
    close_dac();
    free(pcm); free(out);
    s_playback_started = false;
    vTaskDelete(NULL);
}
esp_err_t audio_output_init(void) {
    const usb_host_config_t usb = { .skip_phy_setup = false, .intr_flags = ESP_INTR_FLAG_LEVEL1 };
    esp_err_t err = usb_host_install(&usb);
    if (err != ESP_OK) return err;
    if (xTaskCreatePinnedToCore(host_events, "usb_host_evt", 4096, NULL, 12, NULL,
                                PLAY_CORE) != pdPASS) return ESP_ERR_NO_MEM;
    const uac_host_driver_config_t cfg = {
        .create_background_task = true, .task_priority = 10,
        .stack_size = 4096, .core_id = PLAY_CORE,
        .callback = on_driver, .callback_arg = NULL,
    };
    err = uac_host_install(&cfg);
    if (err != ESP_OK) return err;
    if (!audio_resample_init(44100, RATE, 2)) return ESP_FAIL;
    sf_peq_bridge_init(RATE);
    ESP_LOGI(TAG, "USB Audio Host initialized, waiting for DAC");
    return ESP_OK;
}
void audio_output_start(void) {
    if (s_playback_started) return;
    s_running = true;
    if (xTaskCreatePinnedToCore(playback, "d07_play", 6144, NULL,
                                AUDIO_PLAYBACK_TASK_PRIORITY, NULL, PLAY_CORE) == pdPASS)
        s_playback_started = true;
}
void audio_output_stop(void) { s_running = false; }
void audio_output_flush(void) { s_flush = true; }
void audio_output_set_source_rate(int rate) {
    if (rate > 0 && rate != s_source_rate) {
        s_source_rate = rate;
        s_new_rate = true;
    }
}
void audio_output_set_sample_rate(uint32_t rate) { (void)rate; }
esp_err_t audio_output_write(const void *data, size_t bytes, TickType_t wait) {
    (void)data; (void)bytes; (void)wait;
    return ESP_ERR_NOT_SUPPORTED; // A2DP / raw I2S writer is not a USB writer.
}
uint32_t audio_output_get_hardware_latency_us(void) { return 30000; }
