#include <iostream>
#include <fstream>
#include "mpcdados.hpp"
#include "funcoes_auxiliares.hpp"

#define INF 999.9f
#define pi 3.14159f

// Construtor da Classe MPCDados
MPCDados::MPCDados() {

	std::cout << "== Building MPC Class == \n" << std::endl;
	// Sampling time (seconds)
	this->tau = 1.0/50.0;
	
	std::cout << "	== Loading Model Matrices == \n" << std::endl;

	// Configuração inicial dos dados do MPC
	this->A.resize(6,6);
   	this->A << 1.0, this->tau, 0.0, 0.0, 0.0, 0.0,
	           0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 
			   0.0, 0.0, 1.0, this->tau, 0.0, 0.0,
			   0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 
			   0.0, 0.0, 0.0, 0.0, 1.0, this->tau, 
			   0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

    this->B.resize(6,3); 
	// MODELO AHRSUS
    //this->B << 0.0001, 0.0, 0.0,
	           //0.1963, 0.0, 0.0, 
			   //0.0, 0.0001, 0.0,
			   //0.0, 0.0618, 0.0, 
			   //0.0, 0.0, 0.0002,
			   //0.0, 0.0, 0.3439;
			   
	// MODELO F450		   
	this->B <<  0.00443459,0.00000000,0.00000000,
				0.44345898,0.00000000,0.00000000,
				0.00000000,0.00433839,0.00000000,
				0.00000000,0.43383948,0.00000000,
				0.00000000,0.00000000,0.01176471,
				0.00000000,0.00000000,1.17647059; 
				
	this->MAX_TORQUE = 10.0;
  
    //this->Cr.resize(3,6); 
    //this->Cr << 1.0,0.0,0.0,0.0,0.0,0.0,
				//0.0,0.0,1.0,0.0,0.0,0.0,
				//0.0,0.0,0.0,0.0,1.0,0.0;
				
	this->Cr.resize(6,6); 
    this->Cr << 1.0,0.0,0.0,0.0,0.0,0.0,
				0.0,1.0,0.0,0.0,0.0,0.0,
				0.0,0.0,1.0,0.0,0.0,0.0,
				0.0,0.0,0.0,1.0,0.0,0.0,
				0.0,0.0,0.0,0.0,1.0,0.0,
				0.0,0.0,0.0,0.0,0.0,1.0;
				
	// Constrained States
    this->Cc.resize(2,6); // Dimensoes da Matriz Cc (linhas,colunas)
    this->Cc << 1.0,0.0,0.0,0.0,0.0,0.0,
                0.0,0.0,1.0,0.0,0.0,0.0;
				
	// Prediction Horizon
    //this->N = 10; // MODELO AHRSUS SEM PARAMETRIZAÇÃO
    //this->N = 30; // BOM RESULTADO NA PLATAFORMA, TEMPO DE CALCULO PROBLEMATICO?
    this->N = 15;
    
    // Problem Dimensions 
    n = this->B.rows();
	nu = this->B.cols();
	ny = this->Cr.rows();
	nH = this->N*nu;
	nc = this->Cc.rows();
	ntot = this->N*nu;   
	
    // Weighting matrices
    std::cout << "	== Loading Weighting Matrices == \n" << std::endl;

    this->Qy.resize(ny,ny);		
    // MODELO AHRSUS SEM PARAMETRIZAÇÃO		
	//this->Qy << 150.0,0.0,0.0,0.0,0.0,0.0,
				//0.0,1.0,0.0,0.0,0.0,0.0,
				//0.0,0.0,150.0,0.0,0.0,0.0,
				//0.0,0.0,0.0,1.0,0.0,0.0,
				//0.0,0.0,0.0,0.0,1.0,0.0,
				//0.0,0.0,0.0,0.0,0.0,100.0;
	
	// MODELO F450 COM PARAMETRIZAÇÃO
	//this->Qy << 50.0,0.0,0.0,0.0,0.0,0.0,
				//0.0,2.0,0.0,0.0,0.0,0.0,
				//0.0,0.0,100.0,0.0,0.0,0.0,
				//0.0,0.0,0.0,1.0,0.0,0.0,
				//0.0,0.0,0.0,0.0,1.0,0.0,
				//0.0,0.0,0.0,0.0,0.0,10.0;
	
	//SIMULAÇÃO EM AIRSIM			
	this->Qy << 10.0,0.0,0.0,0.0,0.0,0.0,
				0.0,1.0,0.0,0.0,0.0,0.0,
				0.0,0.0,10.0,0.0,0.0,0.0,
				0.0,0.0,0.0,1.0,0.0,0.0,
				0.0,0.0,0.0,0.0,1.0,0.0,
				0.0,0.0,0.0,0.0,0.0,100.0;
				
    this->Qu.resize(nu,nu);
    //GAZEBO	
    this->Qu << 0.1,0.0,0.0,
                0.0,0.1,0.0,
                0.0,0.0,0.1;
    // AIRSIM                
    this->Qu << 1.0,0.0,0.0,
				0.0,1.0,0.0,
                0.0,0.0,1.0;


    // ***** ycmin e ycmax Serão vetores *****
    this->ycmin.resize(nc);
    this->ycmax.resize(nc);
    this->ycmin << -45*pi/180, -45*pi/180;
    this->ycmax <<  45*pi/180,  45*pi/180;
    //this->ycmin << -5*pi/180, -5*pi/180;
    //this->ycmax <<  5*pi/180,  5*pi/180;

    // ***** ulast, umin, umax, deltamin, deltamax serão vetores *****
    this->ulast.resize(nu);   
    this->umin.resize(nu);    
    this->umax.resize(nu);    
    this->deltamin.resize(nu);
    this->deltamax.resize(nu); 

    // Definir os valores do vetor com a seguinte notação
    // vetor << a1, a2, a3, ...; 
    this->ulast    <<  0.0, 0.0, 0.0; 
    this->umin     << -MAX_TORQUE,-MAX_TORQUE,-MAX_TORQUE;
    this->umax     <<  MAX_TORQUE, MAX_TORQUE, MAX_TORQUE;
    //this->deltamin << -1,-1,-1;
    //this->deltamax <<  1, 1, 1;
    this->deltamin << -INF,-INF,-INF;
    this->deltamax <<  INF, INF, INF;    

    // ***** utildemax e utildemin serão vetores de tamanho N*nu *****
    this->utildemax.resize(this->N*nu);
    this->utildemin.resize(this->N*nu);
    for(int i = 0; i< this->N; i++) {
        for (int j = 0; j < nu; j++) {
			this->utildemax(nu*i+j) = this->umax(j);
			this->utildemin(nu*i+j) = this->umin(j);
		}
	}	
	
	compute_cost_matrices();
	compute_constraints_matrices();

    this->F.resize(this->N*this->nu,1); 
	this->F = MatrizX::Zero(this->N*this->nu,1); 

	this->Bineq.resize(this->G1.rows(), 1);
	this->Bineq = MatrizX::Zero(this->G1.rows(), 1);
	
	std::cout << "	== Loading Regulated Output Trajectories == \n" << std::endl;

	// Defining Regulated Output trajectories
	this->test_duration = 240;
	int qtd_dados = trunc(this->test_duration/this->tau) + 1;
	this->yref.resize(qtd_dados,this->ny);
	this->yref = MatrizX::Zero(qtd_dados,this->ny);
	//this->yref.block(0,0,qtd_dados,1) = MatrizX::Ones(qtd_dados,1)*1.5; // Roll ref
	//this->yref.block(0,0,qtd_dados,1) = MatrizX::Zero(qtd_dados,1); // Roll ref	
	//this->yref.block(0,1,qtd_dados,1) = MatrizX::Zero(qtd_dados,1); // Pitch ref
	//this->yref.block(0,2,qtd_dados,1) = MatrizX::Zero(qtd_dados,1); // Yaw ref
		
	std::cout << "	== Generating Yref == \n" << std::endl;	
	// Concatenating the trajectories
	this->tilde_yref.resize((qtd_dados)*this->ny);
	this->tilde_yref = tilde(yref);
	
	// Prealocating the trajectory projection vector
	this->yref_pred.resize(this->N*this->ny);
	this->yref_pred = MatrizX::Zero(this->N*this->ny,1);
}

int MPCDados::compute_cost_matrices() {	
	
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

int MPCDados::compute_constraints_matrices() {
	// Prepara Variáveis
	int ind1,ind2;
	
	// Matrizes Temporárias
	MatrizX inter_Psi_i, Phi_i, Psi_i;

	MatrizX G1_1(this->N*nc,n);
	
	MatrizX G2_2(nu+(this->N-1)*nu, nu);		

	MatrizX G3_11(this->N*nc,1);	
	
	MatrizX G3_12(this->N*nc,1); 
	
	MatrizX G3_21(this->N*nu,1);   
	
	MatrizX G3_22(this->N*nu,1); 
	
	MatrizX Aineq_1(this->N*nc, this->N*nu);  // OK **** (N*nc, N*nu)
	MatrizX Aineq_2(this->N*nu, this->N*nu);  // OK **** (N*nu, N*nu)
	MatrizX interPinuN(ntot,ntot);
	MatrizX Pi_nuN(nu,this->N*nu);
	MatrizX Cc_Psi_Dc_PinuN(nc,this->N);
	MatrizX Inter_Psi_A;

	G1_1 = MatrizX::Zero(this->N*nc,n);
	G2_2 = MatrizX::Zero(nu+(this->N-1)*nu, nu);
	G3_11 = MatrizX::Zero(this->N*nc,1);
	G3_12 = MatrizX::Zero(this->N*nc,1);
	G3_21 = MatrizX::Zero(this->N*nu,1);
	G3_22 = MatrizX::Zero(this->N*nu,1);
	interPinuN = MatrizX::Zero(ntot,ntot);
	Aineq_1 = MatrizX::Zero(this->N*nc, this->N*nu);
	Aineq_2 = MatrizX::Zero(this->N*nu, this->N*nu);
	
	MatrizX Id(nu,nu);
	MatrizX Id_neg(nu,nu);
	Id = MatrizX::Identity(nu,nu);
	Id_neg = -Id;

	Phi_i = this->A;
	inter_Psi_i = this->B;
	
	// Setting a direct input matrix	
	if(this->Dc.rows() == 0) {
		this->Dc.resize(this->Cc.rows(),this->B.cols());
		this->Dc = MatrizX::Zero(this->Cc.rows(),this->B.cols());
	}
	
	Psi_i.resize(n,this->N*nu);
	Psi_i = MatrizX::Zero(n,this->N*nu);
	
	// Laço 
	for (int i = 0; i < this->N; i++) {
		Psi_i = mescla_matriz(inter_Psi_i, Psi_i, 0, 0);		

		// Verificar os indices abaixo
		Pi_nuN = interPinuN.block(i*nu,0,nu,this->N*nu);
		
		Cc_Psi_Dc_PinuN = this->Cc*Psi_i+this->Dc*Pi_nuN;		

		Aineq_1 = mescla_matriz(Cc_Psi_Dc_PinuN, Aineq_1, i*nc, 0); 

		// Calculos dos indices para substituir nas matrizes
		ind1 = (i*nu);

		Aineq_2 = mescla_matriz(Id, Aineq_2, ind1, ind1);
		
		if(i>0){
			ind2 = (i-1)*nu;
			Aineq_2 = mescla_matriz(Id_neg, Aineq_2, ind1, ind2);
		}		

		G1_1 = mescla_matriz(-(this->Cc*Phi_i), G1_1, i*nc, 0);
		for (int j = 0; j < nc; j++) {
			G3_11(i*nc+j) = this->ycmax(j);
			G3_12(i*nc+j) = -this->ycmin(j);
		}

		for (int j = 0; j < nu; j++) {
			G3_21(i*nu+j) = this->deltamax(j);
			G3_22(i*nu+j) = -this->deltamin(j);
		}

		Phi_i = Phi_i*this->A;
		Inter_Psi_A = this->A*inter_Psi_i;
		inter_Psi_i.resize(this->B.rows(), inter_Psi_i.cols()+nu);
		inter_Psi_i = concat_horizontal_matriz(Inter_Psi_A, this->B);
	}
		
	this->Aineq.resize(Aineq_1.rows()*2+Aineq_2.rows()*2, Aineq_1.cols());
	this->Aineq = MatrizX::Zero(Aineq_1.rows()*2+Aineq_2.rows()*2, Aineq_1.cols());		

	this->Aineq = mescla_matriz(Aineq_1, this->Aineq, 0, 0);
	this->Aineq = mescla_matriz(-Aineq_1, this->Aineq, Aineq_1.rows(), 0);
	this->Aineq = mescla_matriz(Aineq_2, this->Aineq, Aineq_1.rows()*2, 0);
	this->Aineq = mescla_matriz(-Aineq_2, this->Aineq, Aineq_1.rows()*2+Aineq_2.rows(), 0);		

	this->G1.resize(G1_1.rows()*2+this->N*2*nu,n);
	this->G1 = MatrizX::Zero(G1_1.rows()*2+this->N*2*nu,n);
	this->G1 = mescla_matriz(G1_1, this->G1, 0, 0);
	this->G1 = mescla_matriz(-G1_1, this->G1, G1_1.rows(), 0);		
	
	G2_2 = mescla_matriz(MatrizX::Identity(nu,nu), G2_2, 0, 0);
	this->G2.resize(2*this->N*nc+G2_2.rows()*2,nu);
	this->G2 = MatrizX::Zero(2*this->N*nc+G2_2.rows()*2,nu);
	this->G2 = mescla_matriz(G2_2, this->G2, 2*this->N*nc, 0);
	this->G2 = mescla_matriz(-G2_2, this->G2, 2*this->N*nc+G2_2.rows(), 0);		

	this->G3.resize(G3_11.rows()+G3_12.rows()+G3_21.rows()+G3_22.rows(),G3_11.cols());
	this->G3 = MatrizX::Zero(G3_11.rows()+G3_12.rows()+G3_21.rows()+G3_22.rows(),G3_11.cols());
	this->G3 = mescla_matriz(G3_11, this->G3, 0, 0);
	this->G3 = mescla_matriz(G3_12, this->G3, G3_11.rows(), 0);
	this->G3 = mescla_matriz(G3_21, this->G3, G3_11.rows()+G3_12.rows(), 0);
	this->G3 = mescla_matriz(G3_22, this->G3, G3_11.rows()+G3_12.rows()+G3_21.rows(), 0);	

	return 0;

}

VetorX MPCDados::tilde(MatrizX A)
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

void MPCDados::imprime_MPC() {
	using namespace std;

	cout << "======Imprime MPC=====" << endl;
	
	cout << "Matrizes" << endl;
	cout << "MPC->A("  << A.rows()    << "," << A.cols()    << ")" << endl;
	cout << "MPC->B("  << B.rows()    << "," << B.cols()    << ")" << endl;
	cout << "MPC->Cr(" << Cr.rows()   << "," << Cr.cols()   << ")" << endl;
	cout << "MPC->Cc(" << Cc.rows()   << "," << Cc.cols()   << ")" << endl;
	cout << "MPC->H("  << H.rows()    << "," << H.cols()    << ")" << endl;
	cout << "MPC->F1(" << F1.rows()   << "," << F1.cols()   << ")" << endl;
	cout << "MPC->F2(" << F2.rows()   << "," << F2.cols()   << ")" << endl;
	cout << "MPC->F3(" << F3.rows()   << "," << F3.cols()   << ")" << endl;
	cout << "MPC->G1(" << G1.rows()   << "," << G1.cols()   << ")" << endl;
	cout << "MPC->G2(" << G2.rows()   << "," << G2.cols()   << ")" << endl << G2 << endl;
	cout << "MPC->G3(" << G3.rows()   << "," << G3.cols()   << ")" << endl << G3 << endl;
	cout << "MPC->Aineq("     << Aineq.rows()     << "," << Aineq.cols()     << ")" << endl;
	cout << "MPC->utildemax(" << utildemax.rows() << "," << utildemax.cols() << ")" << endl;
	cout << "MPC->utildemax(" << utildemin.rows() << "," << utildemin.cols() << ")" << endl;
	
	/*
	cout << "Variaveis" << endl;
	cout << "MPC->Qy       :" << Qy       << endl;
	cout << "MPC->Qu       :" << Qu       << endl;
	cout << "MPC->N        :" << N        << endl;
	cout << "MPC->ycmin    :" << ycmin    << endl;
	cout << "MPC->ycmax    :" << ycmax    << endl;
	cout << "MPC->umin     :" << umin     << endl;
	cout << "MPC->umax     :" << umax     << endl;
	cout << "MPC->ulast    :" << ulast    << endl;
	cout << "MPC->deltamin :" << deltamin << endl;
	cout << "MPC->deltamax :" << deltamax << endl;
	*/
}

