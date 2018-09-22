#ifndef FUNCOES_AUXULIARES_H
#define FUNCOES_AUXULIARES_H

#include "definicoes.hpp"

MatrizX P_i(int i, int n1, int N);

// Mescla Matriz A na Matriz B (Copia os dados da Matriz A para a B iniciando na linha row e coluna col)
MatrizX mescla_matriz(MatrizX A, MatrizX B, int row, int col);

MatrizX concat_horizontal_matriz(MatrizX A, MatrizX B);

MatrizX max_matrix(_real n, MatrizX A);

void imprime_no_arquivo(std::ofstream *file, MatrizX m, std::string name);

#endif