#ifndef DEFINICOES_H
#define DEFINICOES_H

#include <Eigen/Dense>
#include <string>

//#define M_FLOAT
#define M_DOUBLE

using namespace Eigen;

#ifdef M_FLOAT
typedef Eigen::Matrix<float, Dynamic, Dynamic> MatrizX;
typedef Matrix<float, Dynamic, 1> VetorX;
typedef float _real;
#define MAX_VALUE FLT_MAX
#else 
#ifdef M_DOUBLE
typedef Eigen::Matrix<double, Dynamic, Dynamic> MatrizX;
typedef Matrix<double, Dynamic, 1> VetorX;
typedef double _real;
#define MAX_VALUE DBL_MAX
#endif
#endif

#endif