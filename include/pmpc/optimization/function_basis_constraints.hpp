#pragma once


#include <pmpc/macros.hpp>
#include <pmpc/controls/function_basis.hpp>
#include <pmpc/optimization/linear_control_constraints.hpp>


namespace pmpc {
namespace optimization {

/// Class that represents linear constraints on the control inputs.
template<class Scalar>
class FunctionBasisConstraints : public LinearControlConstraints<Scalar> {
public:
/// \cond
  // Allows the use of VectorXs and MatrixXs
  PMPC_MATRIX_VECTOR_TYPEDEFS(Scalar);
/// \endcond

  FunctionBasisConstraints(
    const controls::FunctionBasis<Scalar>& fbparam,
    const std::vector<int>& constrained_samples
  );

  /// Internally update all matrices related to the constraints.
  virtual void update(
    const Eigen::Ref<const VectorXs> p
  ) override;

  const controls::FunctionBasis<Scalar>& fbparam;

protected:
  using LinearControlConstraints<Scalar>::Cb;
  using LinearControlConstraints<Scalar>::db;
  using LinearControlConstraints<Scalar>::Cv;
  using LinearControlConstraints<Scalar>::dv;
  using LinearControlConstraints<Scalar>::C;
  using LinearControlConstraints<Scalar>::d;
  using LinearControlConstraints<Scalar>::constraints_changed;
  std::vector<int> samples;
};

} // namespace optimization
} // namespace pmpc

