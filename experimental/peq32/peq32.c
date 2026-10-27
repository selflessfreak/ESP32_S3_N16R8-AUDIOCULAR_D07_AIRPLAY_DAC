#include "peq32.h"
#include <math.h>
#include <string.h>

void sf_peq32_init(sf_peq32_t *e, int sample_rate) {
  memset(e, 0, sizeof(*e));
  e->sample_rate = sample_rate;
  e->preamp_gain = 1.0f;
}

bool sf_peq32_set_preamp(sf_peq32_t *e, float db) {
  if (!e || !isfinite(db) || db < -48.0f || db > 0.0f) return false;
  e->preamp_gain = powf(10.0f, db / 20.0f);
  return true;
}

bool sf_peq32_set(sf_peq32_t *e, size_t index, const sf_peq_filter_t *f) {
  if (!e || !f || index >= SF_PEQ_BANDS || e->sample_rate <= 0) return false;
  sf_peq_section_t *s = &e->sections[index];
  if (!f->enabled) { s->enabled = false; memset(s->z1,0,sizeof(s->z1)); memset(s->z2,0,sizeof(s->z2)); return true; }
  if (!isfinite(f->frequency_hz) || !isfinite(f->gain_db) || !isfinite(f->q) ||
      f->frequency_hz < 20 || f->frequency_hz >= 0.49f*e->sample_rate ||
      f->gain_db < -24 || f->gain_db > 24 || f->q < 0.3f || f->q > 12 ||
      f->type < SF_PEQ_PEAK || f->type > SF_PEQ_HIGH_SHELF) return false;
  const double pi = 3.14159265358979323846;
  double w = 2*pi*f->frequency_hz/e->sample_rate;
  double c = cos(w), sn = sin(w), A = pow(10.0, f->gain_db/40.0);
  double alpha = sn/(2.0*f->q);
  double b0,b1,b2,a0,a1,a2;
  if (f->type == SF_PEQ_PEAK) {
    b0=1+alpha*A; b1=-2*c; b2=1-alpha*A;
    a0=1+alpha/A; a1=-2*c; a2=1-alpha/A;
  } else {
    /* Shelf slope S=1.0; Q is not a shelf parameter in this implementation.
       Hardware UI should label this distinction clearly. */
    double aa=sn/sqrt(2.0), t=2*sqrt(A)*aa;
    if (f->type == SF_PEQ_LOW_SHELF) {
      b0=A*((A+1)-(A-1)*c+t); b1=2*A*((A-1)-(A+1)*c);
      b2=A*((A+1)-(A-1)*c-t); a0=(A+1)+(A-1)*c+t;
      a1=-2*((A-1)+(A+1)*c); a2=(A+1)+(A-1)*c-t;
    } else {
      b0=A*((A+1)+(A-1)*c+t); b1=-2*A*((A-1)+(A+1)*c);
      b2=A*((A+1)+(A-1)*c-t); a0=(A+1)-(A-1)*c+t;
      a1=2*((A-1)-(A+1)*c); a2=(A+1)-(A-1)*c-t;
    }
  }
  if (!(a0>0) || !isfinite(a0)) return false;
  s->b0=(float)(b0/a0); s->b1=(float)(b1/a0); s->b2=(float)(b2/a0);
  s->a1=(float)(a1/a0); s->a2=(float)(a2/a0);
  memset(s->z1, 0, sizeof(s->z1)); memset(s->z2, 0, sizeof(s->z2));
  s->enabled=true;
  return true;
}

void sf_peq32_process(sf_peq32_t *e, float *interleaved, size_t frames) {
  if (!e || !interleaved) return;
  for (size_t i=0;i<frames;i++) {
    for (int ch=0;ch<2;ch++) {
      float x=interleaved[i*2+ch]*e->preamp_gain;
      for (int j=0;j<SF_PEQ_BANDS;j++) {
        sf_peq_section_t *s=&e->sections[j];
        if (!s->enabled) continue;
        /* Transposed direct form II; float internal state. */
        float y=s->b0*x+s->z1[ch];
        s->z1[ch]=s->b1*x-s->a1*y+s->z2[ch];
        s->z2[ch]=s->b2*x-s->a2*y;
        x=y;
      }
      interleaved[i*2+ch]=x;
    }
  }
}
