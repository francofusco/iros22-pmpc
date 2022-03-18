#pragma once

#include <pmpc/macros.hpp>
#include <pmpc/models/continuous/linear.hpp>

namespace pmpc {
namespace models {
namespace continuous {


/// Definition of a generic n-order linear integrator.
template<class Scalar>
class Integrator : public Linear<Scalar> {
public:
  // Allows the use of VectorXs and MatrixXs
  PMPC_MATRIX_VECTOR_TYPEDEFS(Scalar);

  using Linear<Scalar>::A;
  using Linear<Scalar>::B;

  /// Creates a linear integrator of given order.
  Integrator(int order) : Integrator<Scalar>(order, 1) { }

  Integrator(int order, int dim)
  : Linear<Scalar>(order*dim, dim)
  {
    const auto size = dim*(order-1);
    A.topRightCorner(size, size) = MatrixXs::Identity(size, size);
    B.bottomRows(dim) = MatrixXs::Identity(dim,dim);
  }
};


} // continuous
} // models
} // pmpc
