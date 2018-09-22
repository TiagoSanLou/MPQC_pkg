#ifndef EXPPARAM_H
#define EXPPARAM_H

#include <float.h>		
#include "definicoes.hpp"
#include "mpcdados.hpp"
#include "simdados.hpp"

class expParam {
public:	
	int N, np;
	_real tau;
	_real alpha;

	VetorX tr;
	VetorX ne;	
	
	MatrizX Pi_e, Hr, Fr, Ar, Br;

public:
	expParam(MPCDados *MPC);
	int compute_Pi_e(MPCDados *MPC);
	int compute_reduced_matrices(MPCDados *MPC);
};

#endif
