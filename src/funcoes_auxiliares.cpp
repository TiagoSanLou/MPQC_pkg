#include <fstream>
#include "funcoes_auxiliares.hpp"

MatrizX P_i(int i, int n1, int N) {
	MatrizX f(n1,N*n1), Id(n1,n1);
	int j, k, l;
	i=i+1;
	f = MatrizX::Zero(n1,N*n1);
	Id = MatrizX::Identity(n1,n1);
	for (j = 0; j < n1; ++j) {
		l = 0;
		for (k = (i-1)*n1; k < (i*n1); ++k) {
			f(j,k) = Id(j,l++);
		}
	}
	return f;
}

// Mescla Matriz A na Matriz B (Copia os dados da Matriz A para a B iniciando na linha row e coluna col)
MatrizX mescla_matriz(MatrizX A, MatrizX B, int row, int col) {
	int i, j, k, l;
	MatrizX C(B.rows(),B.cols());
	C = B;
	k = 0;
	for (i = row; i < (A.rows()+row); i++) 	{
		l = 0;
		for (j = col; j < (A.cols()+col); j++) {
			C(i,j) = A(k,l++);
		}
		k++;
	}
	return C;
}

MatrizX concat_horizontal_matriz(MatrizX A, MatrizX B) {
	int i, j, k;
	MatrizX C(A.rows(),A.cols()+B.cols());
	C = MatrizX::Zero(A.rows(),A.cols()+B.cols());
	for (i = 0; i < A.rows(); i++) 	{
		for (j = 0; j < A.cols(); j++) {
			C(i,j) = A(i,j);
		}
	}
	for (i = 0; i < A.rows(); i++) 	{
		k = 0;
		for (j = A.cols(); j < (A.cols() + B.cols()); j++) {
			C(i,j) = B(i,k++);
		}
	}

	return C;
}

MatrizX max_matrix(_real n, MatrizX A){
	MatrizX B(A.rows(), A.cols());
	for (int i = 0; i < A.rows(); ++i) {
		for (int j = 0; j < A.cols(); ++j) {
			if(A(i,j) < n) 
				B(i,j) = n;
			else
				B(i,j) = A(i,j);
		}
	}
	return B;
}

void imprime_no_arquivo(std::ofstream *file, MatrizX m, std::string name) {
	*file << name << " = [";
	for (int i = 0; i < m.rows(); ++i) {
		for (int j = 0; j < m.cols(); ++j) {
			*file << std::scientific << m(i,j) << " ";
		}
		*file << ";" << std::endl << "\t";
	}
	*file << "];" << std::endl;
}
