#ifndef PARAMETRIZACAO_H
#define PARAMETRIZACAO_H

#include "definicoes.hpp"

// Matrizes do Modelo
MatrizX A(3,3); // [0 1 0;0 0 1;0 0 0]
		A <<  1.0, 0.1, 0.005,
     	 	  0.0, 1.0, 0.100,
     	 	  0.0, 0.0, 1.000;

MatrizX B(3,1); // [0;0;1]
		B << 0.000166666666666667, 0.005, 0.1;


MatrizX Cr(1,3); // [1 0 0]
		Cr << 1.0, 0.0, 0.0;

// Pesos
_real Qy = 1e5; 
_real Qu = 0.1;

// Restricoes
MatrizX Cc(2,3); // [1 0 0; 0 1 0]
		Cc << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0;

_real ycmin[2] = [-inf;-2];
_real ycmax[2] = [inf;2];
_real umin     = -30;
_real umax     = 30;
_real ulast    = 0;
_real deltamin = -50;
_real deltamax = 50;

// Periodo de Amostragem
_real  tau = 0.1;
// Tempo de Simulação
int tsim = 20;
// Horizonte de Predição
int N = 20;


#endif