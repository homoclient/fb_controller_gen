#include "simple_linalg.h"
#include <math.h>
#include <string.h>
#include <assert.h>

void vecnegated(double *v, unsigned int len)
{
  for (unsigned int i=0; i<len; i++) v[i] = -v[i];
}

void vecscaled(double *v, double s, unsigned int len)
{
  for (unsigned int i=0; i<len; i++) v[i] *= s;
}

void vecsumd(double *res, double *v1, double *v2, unsigned int len)
{
  for (unsigned int i=0; i<len; i++) res[i] = v1[i]+v2[i];
}

void vecminusd(double *res, double *v1, double *v2, unsigned int len)
{
  for (unsigned int i=0; i<len; i++) res[i] = v1[i]-v2[i];
}

void transposed(double *res, double *m, unsigned int hm, unsigned int wm)
{
  for (unsigned int i=0; i<hm; i++) {
    for (unsigned int j=0; j<wm; j++) {
      res[i+j*hm] = m[j+i*wm];
    }
  }
}

void matrowscaled(double *res, double *m, const double *diag, unsigned int hm, unsigned int wm)
{
  for (unsigned int i=0; i<hm; i++) {
    double k = diag[i];
    for (unsigned int j=0; j<wm; j++) {
      res[j+i*wm] = k*m[j+i*wm];
    }
  }
}

void matcolscaled(double *res, double *m, const double *diag, unsigned int hm, unsigned int wm)
{
  for (unsigned int i=0; i<wm; i++) {
    double k = diag[i];
    for (unsigned int j=0; j<hm; j++) {
      res[i+j*wm] = k*m[i+j*wm];
    }
  }
}

void matvecmuld(double *res, double *m, double *v, unsigned int hm, unsigned int wm)
{
  for (unsigned int i=0; i<hm; i++) res[i] = 0.0;
  for (unsigned int i=0; i<wm; i++) {
    double k = v[i];
    for (unsigned int j=0; j<hm; j++) {
      res[j] += k*m[i+j*wm];
    }
  }
}

void matmuld(double *res, double *a, double *b, unsigned int ha, unsigned int wa, unsigned int hb, unsigned int wb)
{
  assert(wa == hb);
  for (unsigned int i=0; i<ha*wb; i++) {
    res[i] = 0.0;
  }
  for (unsigned int i=0; i<wa; i++) {
    for (unsigned int j=0; j<ha; j++) {
      double k = a[i+wa*j];
      for (unsigned int t=0; t<wb; t++) {
        res[t+j*wb] += k*b[t+i*wb];
      }
    }
  }
}

#define ZERO_THRESHOLD 1e-6

int invd_gaussian(double *res, double *m, unsigned int dim, double *tmp, unsigned int *index_tmp)
{
  for (unsigned int i=0; i<dim; i++) index_tmp[i] = i;
  for (unsigned int col=0; col<dim; col++) {
    unsigned int pivot_row = 0;
    double pivot = 0.0;
    for (unsigned int i=col; i<dim; i++) {
      double v = fabs(m[i*dim + col]);
      if (v > pivot) {
        pivot = v;
        pivot_row = i;
      }
    }
    if (pivot < ZERO_THRESHOLD) return -1; // pivoting failed
    unsigned int t = index_tmp[col];
    index_tmp[col] = index_tmp[pivot_row];
    index_tmp[pivot_row] = t;
    memcpy((void *)tmp, (void *)(m+pivot_row*dim), dim*sizeof(double));
    memcpy((void *)(m+pivot_row*dim), (void *)(m+col*dim), dim*sizeof(double));
    memcpy((void *)(m+col*dim), (void *)tmp, dim*sizeof(double));

    pivot = m[col+col*dim];
    double pivot_inv = 1.0/pivot;
    for (unsigned int i=0; i<dim; i++) {
      if (i != col) m[col+i*dim] *= -pivot_inv;
      else m[col+i*dim] = pivot_inv;
    }
    for (unsigned int i=0; i<dim; i++) {
      if (i==col) continue;
      for (unsigned int j=0; j<dim; j++) {
        if (j==col) continue;
        m[j+i*dim] += m[j+col*dim]*m[col+i*dim];
      }
    }
    for (unsigned int i=0; i<dim; i++) {
      if (i==col) continue;
      m[i+col*dim] *= pivot_inv;
    }
  }
  for (unsigned int i=0; i<dim; i++) {
    for (unsigned int j=0; j<dim; j++) {
      res[index_tmp[j]+i*dim] = m[j+i*dim];
    }
  }
  return 0;
}

void printmat(FILE *fp, double *m, unsigned int hm, unsigned int wm)
{
  for (unsigned int i=0; i<hm; i++) {
    for (unsigned int j=0; j<wm; j++) {
      fprintf(fp, "%.2f\t", m[j+i*wm]);
    }
    putc('\n', fp);
  }
}

void printvec(FILE *fp, double *v, unsigned int len)
{
  for (unsigned int i=0; i<len; i++) {
    fprintf(fp, "%.2f\t", v[i]);
  }
  putc('\n', fp);
}