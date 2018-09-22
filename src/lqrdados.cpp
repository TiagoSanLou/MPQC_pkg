#include <iostream>
#include <fstream>
#include "lqrdados.hpp"
#include "funcoes_auxiliares.hpp"

// Construtor da Classe LQRDados
LQRDados::LQRDados() {
	std::cout << "== Building LQR Class == \n" << std::endl;

	// Sampling time (seconds)
	this->tau = 1.0/50.0;
	
	// Configuração inicial dos dados do MPC
	std::cout << "	== Loading Model Matrices == \n" << std::endl;
	this->A.resize(6,6);  // Dimensoes da Matriz A (linhas,colunas)
   	this->A << 1.0, this->tau, 0.0, 0.0, 0.0, 0.0,
	           0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 
			   0.0, 0.0, 1.0, this->tau, 0.0, 0.0,
			   0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 
			   0.0, 0.0, 0.0, 0.0, 1.0, this->tau, 
			   0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

    this->B.resize(6,3); // Dimensoes da Matriz B (linhas,colunas)
	
    this->B << 0.0001, 0.0, 0.0,
	           0.1963, 0.0, 0.0, 
			   0.0, 0.0001, 0.0,
			   0.0, 0.0618, 0.0, 
			   0.0, 0.0, 0.0002,
			   0.0, 0.0, 0.3439;		  
  
    this->Cr.resize(6,6); // Dimensoes da Matriz Cr (linhas,colunas)
    this->Cr << 1.0,0.0,0.0,0.0,0.0,0.0,
				0.0,1.0,0.0,0.0,0.0,0.0,
                0.0,0.0,1.0,0.0,0.0,0.0,
                0.0,0.0,0.0,1.0,0.0,0.0,
                0.0,0.0,0.0,0.0,1.0,0.0,
                0.0,0.0,0.0,0.0,0.0,1.0;
	
	// Prediction Horizon
    this->N = 50;

	n = this->B.rows();
	nu = this->B.cols();
	ny = this->Cr.rows();
	nH = this->N*nu;
	ntot = this->N*nu;
		
    // Weighting matrices
    std::cout << "	== Loading Weighting Matrices == \n" << std::endl;
    this->Qy.resize(ny,ny);	
    this->Qy << 2.0,0.0,0.0,0.0,0.0,0.0,
                0.0,10.0,0.0,0.0,0.0,0.0,
                0.0,0.0,2.0,0.0,0.0,0.0,
                0.0,0.0,0.0,10.0,0.0,0.0,
                0.0,0.0,0.0,0.0,1.0,0.0,
                0.0,0.0,0.0,0.0,0.0,5.0;
				
    this->Qu.resize(nu,nu);	
    this->Qu << 1.0,0.0,0.0,
                0.0,1,0.0,
                0.0,0.0,1.0;
                
	compute_cost_matrices();
	
	// Computing LQR Gains
	K_N = -P_i(1,nu,N)*this->H.inverse()*this->F1;
	G_N = -P_i(1,nu,N)*this->H.inverse()*this->F2;
	L_N = -P_i(1,nu,N)*this->H.inverse()*this->F3;
	
	//
	this->ulast.resize(nu); 
    this->ulast    <<  0.0, 0.0, 0.0; 

	// Defining Regulated Output trajectories
	std::cout << "	== Loading Regulated Output Trajectories == \n" << std::endl;
	int test_duration = 120;
	int qtd_dados = trunc(test_duration/this->tau) + 1;
	this->yref.resize(qtd_dados,this->ny);
	this->yref = MatrizX::Zero(qtd_dados,this->ny);
	//this->yref.block(0,0,qtd_dados,1) = MatrizX::Ones(qtd_dados,1)*1.5; // Roll ref
	//this->yref.block(0,0,qtd_dados,1) = MatrizX::Zero(qtd_dados,1); // Roll ref	
	//this->yref.block(0,1,qtd_dados,1) = MatrizX::Zero(qtd_dados,1); // Pitch ref
	//this->yref.block(0,2,qtd_dados,1) = MatrizX::Zero(qtd_dados,1); // Yaw ref
		
	// Concatenating the trajectories
	this->tilde_yref.resize((qtd_dados)*this->ny);
	this->tilde_yref = tilde(yref);
	
	// Prealocating the trajectory projection vector
	this->yref_pred.resize(this->N*this->ny);
	this->yref_pred = MatrizX::Zero(this->N*this->ny,1);
}

int LQRDados::compute_cost_matrices() {	
	
	//int n, nu, nH, ny;
	MatrizX inter_Psi_i, Phi_i, Psi_i, Pi_nu_N, Pi_ny_N;
	MatrizX cr_psi_T, A_inter_Psi;
	Phi_i.resize(this->A.rows(), this->A.cols());
	inter_Psi_i.resize(this->B.rows(), this->B.cols());
	A_inter_Psi.resize(this->A.rows(), this->B.cols());

	Phi_i = this->A;
	inter_Psi_i = this->B;

	this->H.resize(nH,nH);
	this->F1.resize(nH,n);
	this->F2.resize(nH,this->N*ny);
	this->F3.resize(nH,nu);

	this->H = MatrizX::Zero(nH,nH);
	this->F1 = MatrizX::Zero(nH,n);
	this->F2 = MatrizX::Zero(nH,this->N*ny);
	this->F3 = MatrizX::Zero(nH,nu);

	Psi_i = MatrizX::Zero(n,nH);
	cr_psi_T.resize(this->Cr.rows(),Psi_i.cols());

	//Matrix calculation loop
	for (int i = 0; i < this->N; i++) {
		Psi_i = mescla_matriz(inter_Psi_i, Psi_i, 0, 0);
		Pi_nu_N = P_i(i, nu, this->N);
		Pi_ny_N = P_i(i, ny, this->N);
		cr_psi_T = this->Cr*Psi_i;
		this->H = this->H+cr_psi_T.transpose()*this->Qy*this->Cr*Psi_i+Pi_nu_N.transpose()*this->Qu*Pi_nu_N;
	    this->F1 = this->F1+Psi_i.transpose()*this->Cr.transpose()*this->Qy*this->Cr*Phi_i;
	    this->F2 = this->F2-Psi_i.transpose()*this->Cr.transpose()*this->Qy*Pi_ny_N;
	    this->F3 = this->F3+Pi_nu_N.transpose()*this->Qu;
		Phi_i = Phi_i*this->A;
		A_inter_Psi = this->A*inter_Psi_i;
		inter_Psi_i.resize(this->B.rows(), inter_Psi_i.cols()+1);
		inter_Psi_i = concat_horizontal_matriz(A_inter_Psi, this->B);
	}
	return 0;

}

VetorX LQRDados::tilde(MatrizX A)
{
	int nt, ny;
	VetorX tilde(A.rows()*A.cols());
	
	nt = A.rows();
	ny = A.cols();
	for (int i=0; i<nt;i++){
		tilde.block(i*ny,0,ny,1) = A.row(i).transpose();
	}	
	return tilde;
	
}

void LQRDados::imprime_LQR() {
	using namespace std;

	cout << "======Imprime MPC=====" << endl;
	
	cout << "Matrizes" << endl;
	cout << "LQR->A("  << A.rows()    << "," << A.cols()    << ") \n" << A  << endl;
	cout << "LQR->B("  << B.rows()    << "," << B.cols()    << ") \n" << B  << endl;
	cout << "LQR->Cr(" << Cr.rows()   << "," << Cr.cols()   << ") \n" << Cr << endl;
	cout << "LQR->H("  << H.rows()    << "," << H.cols()    << ")" << endl;
	cout << "LQR->F1(" << F1.rows()   << "," << F1.cols()   << ")" << endl;
	cout << "LQR->F2(" << F2.rows()   << "," << F2.cols()   << ")" << endl;
	cout << "LQR->F3(" << F3.rows()   << "," << F3.cols()   << ")" << endl;
	cout << "LQR->G1(" << G1.rows()   << "," << G1.cols()   << ")" << endl;
	cout << "LQR->G2(" << G2.rows()   << "," << G2.cols()   << ")" << endl;
	cout << "LQR->G3(" << G3.rows()   << "," << G3.cols()   << ")" << endl;
	
	cout << "LQR->K_N(" << K_N.rows()   << "," << K_N.cols()   << ") \n" << K_N << endl;
	cout << "LQR->G_N(" << G_N.rows()   << "," << G_N.cols()   << ") \n" << G_N << endl;
	cout << "LQR->L_N(" << L_N.rows()   << "," << L_N.cols()   << ") \n" << L_N << endl;
	
	/*
	cout << "Variaveis" << endl;
	cout << "MPC->Qy       :" << Qy       << endl;
	cout << "MPC->Qu       :" << Qu       << endl;
	cout << "MPC->N        :" << N        << endl;
	*/
}

