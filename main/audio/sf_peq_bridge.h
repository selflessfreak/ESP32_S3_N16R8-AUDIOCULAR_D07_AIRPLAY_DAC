#ifndef SF_PEQ_BRIDGE_H
#define SF_PEQ_BRIDGE_H
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "peq32.h"
/* Initialize on playback thread, once at the active sample rate. */
void sf_peq_bridge_init(int sample_rate);
/* Returns false when configuration invalid. Configure before streaming. */
bool sf_peq_bridge_set_band(size_t index, const sf_peq_filter_t *f);
bool sf_peq_bridge_set_preamp(float db);
/* Interleaved stereo int16 processing, in place. */
void sf_peq_bridge_process_i16(int16_t *pcm, size_t frames);
#endif
