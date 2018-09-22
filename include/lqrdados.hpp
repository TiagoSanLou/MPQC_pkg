#ifndef LQRDADOS_H
#define LQRDADOS_H

#include <float.h>		
#include "definicoes.hpp"

class LQRDados {
public:

	_real tau;
	/*
	 * State-Space Model Matrices
	*/ 
	MatrizX A;
	MatrizX B;
	MatrizX Cr;

	// Weighting matrices
	MatrizX Qy; 
	MatrizX Qu; 
	
	// Prediction Horizon
	int N;
	
	/*
	 * Cost Function Matrices
	*/		
	//Hessian H \in Re^{N*nu,N*nu}
	MatrizX H;
	//F1 \in Re^{N*nu,n}; F2 \in Re^{N*nu,N*nr}; F3 \in Re^{N*nu,nu}
	MatrizX F1, F2, F3;
	MatrizX G1, G2, G3;
	MatrizX F;	
	
	/*
	 * LQR GAINS
	*/
	MatrizX K_N, G_N, L_N;	
	
	/*
	 * Regulated Output trajectories
	*/ 
	MatrizX yref;
	VetorX tilde_yref, yref_pred;
	
	VetorX ulast;
	//MatrizX p0;
	
	//Problem Dimensions
	int n, nu, nH, ny, ntot;

public:
	LQRDados();
	int compute_cost_matrices();
	void imprime_LQR();
	VetorX tilde(MatrizX A);
};

#endif
