#pragma once

#include <pmpc/macros.hpp>
#include <pmpc/controls/parameterization.hpp>


namespace pmpc {
namespace optimization {

/// Class that represents generic constraints on the control inputs.
/** \todo Long description.
  * \todo Add an "update" virtual method.
  */
template<class Scalar>
class ControlConstraints {
public:
/// \cond
  // Allows the use of VectorXs and MatrixXs
  PMPC_MATRIX_VECTOR_TYPEDEFS(Scalar);
/// \endcond

  ControlConstraints(const controls::Parameterization<Scalar>& param)
  : param(param)
  { }

  virtual int dim() const = 0;

  /// Evaluate the constraints.
  /** @param[in] p free-parameters of the control.
    * @param[out] g value of the constraints at the current point.
    */
  inline void evaluate(
    const Eigen::Ref<const VectorXs> p,
    Eigen::Ref<VectorXs> g
  ) const
  {
    evaluate_impl(p, g, nullptr);
  }

  /// Evaluate the constraints and their jacobian.
  /** @param[in] p free-parameters of the control.
    * @param[out] g value of the constraints at the current point.
    * @param[out] Jg jacobian of the constraints with respect to the
    *   free-parameters.
    */
  inline void evaluate(
    const Eigen::Ref<const VectorXs> p,
    Eigen::Ref<VectorXs> g,
    Eigen::Ref<MatrixXs> Jg
  ) const
  {
    evaluate_impl(p, g, &Jg);
  }

  const controls::Parameterization<Scalar>& param;

protected:
  virtual void evaluate_impl(
    const Eigen::Ref<const VectorXs> p,
    Eigen::Ref<VectorXs> g,
    Eigen::Ref<MatrixXs>* Jg
  ) const = 0;

};

} // namespace optimization
} // namespace pmpc
