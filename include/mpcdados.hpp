#ifndef MPCDADOS_H
#define MPCDADOS_H

#include <float.h>		
#include "definicoes.hpp"

class MPCDados {
public:

	_real tau;
	_real MAX_TORQUE;
	
	// State-Space Model Matrices
	MatrizX A;
	MatrizX B;
	MatrizX Cr;

	// Weighting matrices
	MatrizX Qy; 
	MatrizX Qu; 
	
	// Prediction Horizon
	int N; 			
	int test_duration;
	
	// Constraint Output Definition Cc \in Re^{nc,n}; yc =  Cc*x
	MatrizX Cc;
	MatrizX Dc;
	
	// Constrained Output Bounds
	VetorX ycmin;
	VetorX ycmax;
	
	// Input Bounds
	VetorX umin, umax;
	VetorX ulast;
	VetorX deltamin, deltamax;
	
	/*
	Cost Function Matrices
	*/	
	//Hessian H \in Re^{N*nu,N*nu}
	MatrizX H;
	//F1 \in Re^{N*nu,n}; F2 \in Re^{N*nu,N*nr}; F3 \in Re^{N*nu,nu}
	MatrizX F1, F2, F3;
	MatrizX G1, G2, G3;
	MatrizX F;
	
	/*
	Constraint Matrices
	*/
	//Aineq \in Re^{2*N*(nc+nu),N*nu}
	MatrizX Aineq;
	MatrizX Bineq;
	VetorX utildemax; // \in Re^{N*nu}
	VetorX utildemin;
	
	// Regulated Output trajectories
	MatrizX yref;
	VetorX tilde_yref, yref_pred;
	
	//MatrizX p0;

	int n, nu, nH, ny, nc, ntot;

public:
	MPCDados();
	int compute_cost_matrices();
	int compute_constraints_matrices();
	void imprime_MPC();
	void exporta_MPC_matlab();
	VetorX tilde(MatrizX A);
};

#endif
