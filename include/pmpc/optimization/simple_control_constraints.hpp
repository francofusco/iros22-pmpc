#pragma once


#include <pmpc/macros.hpp>
#include <pmpc/controls/simple.hpp>
#include <pmpc/optimization/linear_control_constraints.hpp>


namespace pmpc {
namespace optimization {

/// Class that represents linear constraints on the control inputs.
template<class Scalar>
class SimpleControlConstraints : public LinearControlConstraints<Scalar> {
public:
/// \cond
  // Allows the use of VectorXs and MatrixXs
  PMPC_MATRIX_VECTOR_TYPEDEFS(Scalar);
/// \endcond

  SimpleControlConstraints(
    const controls::Simple<Scalar>& simple
  );

  /// Internally update all matrices related to the constraints.
  virtual void update(
    const Eigen::Ref<const VectorXs> p
  ) override;

  const controls::Simple<Scalar>& simple;

protected:
  using LinearControlConstraints<Scalar>::Cb;
  using LinearControlConstraints<Scalar>::db;
  using LinearControlConstraints<Scalar>::Cv;
  using LinearControlConstraints<Scalar>::dv;
  using LinearControlConstraints<Scalar>::C;
  using LinearControlConstraints<Scalar>::d;
  using LinearControlConstraints<Scalar>::constraints_changed;
};

} // namespace optimization
} // namespace pmpc

