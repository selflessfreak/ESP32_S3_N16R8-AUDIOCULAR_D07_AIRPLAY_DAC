/* SF-AirPlay PEQ DSP prototype; based on standard RBJ audio EQ cookbook.
 * MIT-style example code, independent of project's upstream license.
 * DEVELOPMENT COMPONENT ONLY: NOT CONNECTED TO USB FIRMWARE.
 */
#ifndef SF_PEQ32_H
#define SF_PEQ32_H
#include <stdbool.h>
#include <stddef.h>
#define SF_PEQ_BANDS 32
#define SF_PEQ_CHANNELS 2
typedef enum { SF_PEQ_PEAK, SF_PEQ_LOW_SHELF, SF_PEQ_HIGH_SHELF } sf_peq_type_t;
typedef struct { bool enabled; sf_peq_type_t type; float frequency_hz, gain_db, q; } sf_peq_filter_t;
typedef struct { float b0,b1,b2,a1,a2; float z1[2],z2[2]; bool enabled; } sf_peq_section_t;
typedef struct { sf_peq_section_t sections[SF_PEQ_BANDS]; float preamp_gain; int sample_rate; } sf_peq32_t;
void sf_peq32_init(sf_peq32_t *e, int sample_rate);
/* Configure only when playback is stopped or via atomically swapped banks.
 * Returns false on invalid input. Changing filter coefficients resets state.
 */
bool sf_peq32_set(sf_peq32_t *e, size_t index, const sf_peq_filter_t *filter);
bool sf_peq32_set_preamp(sf_peq32_t *e, float db);
/* Interleaved stereo float samples; no hard clipping here. */
void sf_peq32_process(sf_peq32_t *e, float *stereo, size_t frames);
#endif
