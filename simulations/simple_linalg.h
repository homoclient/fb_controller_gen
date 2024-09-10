#ifndef SIMPLE_LINALG_H
#define SIMPLE_LINALG_H

#include <stdio.h>

void vecnegated(double *v, unsigned int len);
void vecsumd(double *res, double *v1, double *v2, unsigned int len);
void vecminusd(double *res, double *v1, double *v2, unsigned int len);
void transposed(double *res, double *m, unsigned int hm, unsigned int wm);
void matrowscaled(double *res, double *m, const double *diag, unsigned int hm, unsigned int wm);
void matcolscaled(double *res, double *m, const double *diag, unsigned int hm, unsigned int wm);
void matvecmuld(double *res, double *m, double *v, unsigned int hm, unsigned int wm);
void matmuld(double *res, double *a, double *b, unsigned int ha, unsigned int wa, unsigned int hb, unsigned int wb);
int invd_gaussian(double *res, double *m, unsigned int dim, double *tmp, unsigned int *index_tmp);

void printmat(FILE *fp, double *m, unsigned int hm, unsigned int wm);
void printvec(FILE *fp, double *v, unsigned int len);

#endif