#pragma once

#include <Eigen/Dense>
#include <unsupported/Eigen/AutoDiff>

/// Macro to define MatrixXs and VectorXs on the template Scalar.
#define PMPC_MATRIX_VECTOR_TYPEDEFS(SCALAR) \
  typedef Eigen::Matrix<SCALAR,Eigen::Dynamic,Eigen::Dynamic> MatrixXs; \
  typedef Eigen::Matrix<SCALAR,Eigen::Dynamic,1> VectorXs;

/// Macro to define ADScalar (an AutoDiff type) and ADVector.
#define PMPC_AUTODIFF_TYPEDEFS(SCALAR) \
  typedef Eigen::AutoDiffScalar<Eigen::Matrix<SCALAR,Eigen::Dynamic,1>> ADScalar; \
  typedef Eigen::Matrix<ADScalar,Eigen::Dynamic,1> ADVector;

/// Macro to define Scalar, MatrixXs, VectorXs, ADScalar and ADVector.
#define PMPC_DEFINE_SCALAR(SCALAR) \
  typedef SCALAR Scalar ; \
  PMPC_MATRIX_VECTOR_TYPEDEFS(SCALAR) \
  PMPC_AUTODIFF_TYPEDEFS(SCALAR)
