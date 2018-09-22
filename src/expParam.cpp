#include <iostream>
#include <math.h> 
#include "expParam.hpp"
#include "funcoes_auxiliares.hpp"

expParam::expParam(MPCDados *MPC) {
	
	int nu = MPC->B.cols();
	int nc = MPC->Cc.rows();
	
	std::cout << "	  == Loading exp parametrization parameters == \n" << std::endl;	
	// Exponential Parametrization parameters
	this->tr.resize(nu);
	this->tr << 0.1, 0.1, 0.1;
	
	this->ne.resize(nu);
    this->ne << 2.0, 2.0, 2.0;
	
	this->alpha = 10;
	
    this->tau = MPC->tau;    
    this->N = MPC->N;		
	
	this->np = 0;	
    for(int i = 0; i < nu; i++){
		this->np = this->np + this->ne(i);
	}
	
	std::cout << "	  == Computing Pi_e == \n" << std::endl;	
	compute_Pi_e(MPC);	
	
	this->Hr.resize(np,np);	
	this->Hr = MatrizX::Zero(np,np); 
	
	this->Fr.resize(np,1);
	this->Fr = MatrizX::Zero(np,1); 
	
	this->Ar.resize(2*this->N*(nc+2*nu),np);
	this->Ar = MatrizX::Zero(2*this->N*(nc+2*nu),np);
	
	this->Br.resize(2*this->N*(nc+2*nu),1);
	this->Br = MatrizX::Zero(2*this->N*(nc+2*nu),1);
}

int expParam::compute_Pi_e(MPCDados *MPC)
{
	MatrizX Mi, Mij;		
	
	int nu = MPC->B.cols();
	int np = 0;
	
    for(int i = 0; i < nu; i++){
		np = np + this->ne(i);
	}
	
	//std::cout << "np = " << np << std::endl;
	
	this->Pi_e.resize(this->N*nu,np);
	this->Pi_e = MatrizX::Zero(this->N*nu,np);
	
	// Loop de Calculo de Pi_e
	Mi.resize(nu,np);
	for(int i = 0; i < this->N; i++){
		Mi = MatrizX::Zero(nu,np);
		for(int j = 0; j < nu; j++){
			Mij.resize(1,this->ne(j));
			Mij = MatrizX::Zero(1,this->ne(j));
			for(int ell = 0; ell < this->ne(j); ell++){
				Mij(0,ell) = exp(-2/this->tr(j)*(i*this->tau)/(ell*alpha+1));
			}
			//std::cout << "Mij = " << Mij << std::endl;
			//std::cout << "Mi = " << Mi << std::endl;
			//std::cout << "j = " << j << std::endl;
			//std::cout << "col = " << col << std::endl;
			int col = 0;
			if(j==0){
				Mi = mescla_matriz(Mij, Mi, 0, 0);
			} else {
				for(int k = 0; k < j; k++){
					col = col + this->ne(k);
				}				
				Mi = mescla_matriz(Mij, Mi, j, col);
			}
            //std::cout << "BREAK 3" << std::endl;			
		}
		this->Pi_e = mescla_matriz(Mi, Pi_e, i*nu, 0);
	} 
	return 0;
}

int expParam::compute_reduced_matrices(MPCDados *MPC){

	this->Hr = this->Pi_e.transpose()*MPC->H*this->Pi_e;
	
	this->Fr = this->Pi_e.transpose()*MPC->F;
	
	this->Ar = mescla_matriz(MPC->Aineq*this->Pi_e, this->Ar,0,0);
	this->Ar = mescla_matriz((-1)*this->Pi_e, this->Ar, MPC->Aineq.rows()-1,0);
	this->Ar = mescla_matriz(this->Pi_e, this->Ar, MPC->Aineq.rows() + this->Pi_e.rows() - 1, 0);
	
	this->Br = mescla_matriz(MPC->Bineq,this->Br,0,0);
	this->Br = mescla_matriz((-1)*MPC->utildemin, this->Br, MPC->Bineq.rows(), 0);
	this->Br = mescla_matriz(MPC->utildemax, this->Br, MPC->Bineq.rows() + MPC->utildemin.rows(), 0); // ERRO!!!!
	
	return 0;
}
