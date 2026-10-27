#include "sf_peq_bridge.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

/* Playback-thread-owned engine; web handlers MUST NOT mutate it directly.
 * Future UI changes must use an atomic copy/swap or playback command queue. */
static sf_peq32_t s_eq;
static bool s_ready;
static float s_buffer[2*512];

void sf_peq_bridge_init(int sample_rate) {
    sf_peq32_init(&s_eq, sample_rate);
    s_ready = true;
}
bool sf_peq_bridge_set_band(size_t index, const sf_peq_filter_t *f) {
    return s_ready && sf_peq32_set(&s_eq, index, f);
}
bool sf_peq_bridge_set_preamp(float db) {
    return s_ready && sf_peq32_set_preamp(&s_eq, db);
}
static int16_t quantize_i16(float value) {
    if (!isfinite(value)) return 0;
    if (value >= 32767.0f) return 32767;
    if (value <= -32768.0f) return -32768;
    return (int16_t)lrintf(value);
}
void sf_peq_bridge_process_i16(int16_t *pcm, size_t frames) {
    if (!s_ready || !pcm) return;
    while (frames) {
        size_t n = frames > 512 ? 512 : frames;
        for (size_t i=0;i<2*n;i++) s_buffer[i] = (float)pcm[i];
        sf_peq32_process(&s_eq, s_buffer, n);
        for (size_t i=0;i<2*n;i++) pcm[i] = quantize_i16(s_buffer[i]);
        pcm += 2*n;
        frames -= n;
    }
}
