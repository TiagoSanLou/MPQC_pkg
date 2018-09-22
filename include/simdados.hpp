#ifndef SIMDADOS_H
#define SIMDADOS_H

#include "definicoes.hpp"
#include "mpcdados.hpp"

class SimDados {
public:
	VetorX x0;
	int tsim, nt;
	_real tau;

	MatrizX lesx;
	VetorX lest, lesIterT;
	MatrizX lesy, lesu;
	
	MatrizX yref;
	VetorX tilde_yref, yref_pred;

public:
	SimDados(MPCDados *MPC);
	VetorX SquareWave(_real periodo, _real min, _real max, int qtd_dados);
	VetorX tilde(MatrizX A);

};

#endif