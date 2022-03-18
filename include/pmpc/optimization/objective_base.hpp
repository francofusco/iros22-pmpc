#pragma once

#include <pmpc/macros.hpp>

namespace pmpc {
namespace optimization {

/// Base class that serves as a common interface for optimization objectives.
template<class Scalar>
class ObjectiveBase {
public:
/// \cond
  // Allows the use of VectorXs and MatrixXs
  PMPC_MATRIX_VECTOR_TYPEDEFS(Scalar);
/// \endcond

  /// Constructor, stores a reference to the given predictor.
  /** @param predictor a predictor instance that allows to perform predictions
    *   for a given model and control parameterization.
    */
  ObjectiveBase(const predictors::Recursive<Scalar>& predictor)
  : predictor(predictor)
  { }

  /// Evaluate the objective of the MPC optimization.
  /** @param[in] x0 initial configuration of the system.
    * @param[in] p free-parameters of the control.
    * @param[out] obj value of the objective function at the current point.
    */
  inline void evaluate(
    const Eigen::Ref<const VectorXs> x0,
    const Eigen::Ref<const VectorXs> p,
    Scalar& obj
  ) const
  {
    evaluate_impl(x0, p, obj, nullptr);
  }

  /// Evaluate the objective of the MPC optimization and its jacobian.
  /** @param[in] x0 initial configuration of the system.
    * @param[in] p free-parameters of the control.
    * @param[out] obj value of the objective function at the current point.
    * @param[out] Jobj jacobian of the objective function with respect to
    *   the free-parameters.
    */
  inline void evaluate(
    const Eigen::Ref<const VectorXs> x0,
    const Eigen::Ref<const VectorXs> p,
    Scalar& obj,
    Eigen::Ref<MatrixXs> Jobj
  ) const
  {
    evaluate_impl(x0, p, obj, &Jobj);
  }

  const predictors::Recursive<Scalar>& predictor; ///< Reference to the predictor used for the current model and control parameterization.

protected:
  /// Low-level method called to evaluate the objective and its jacobian.
  /** This method has to be overridden in sub-classes to allow the evaluation
    * of the objective according to specific implementations.
    * @param[in] x0 initial configuration of the system.
      * @param[in] p free-parameters of the control.
      * @param[out] obj value of the objective function at the current point.
      * @param[out] Jobj jacobian of the objective function with respect to
      *   the free-parameters. If nullptr is given, then the jacobian should
      *   not be computed in the function call.
    */
  virtual void evaluate_impl(
    const Eigen::Ref<const VectorXs> x0,
    const Eigen::Ref<const VectorXs> p,
    Scalar& obj,
    Eigen::Ref<MatrixXs>* Jobj
  ) const = 0;
};

} // namespace optimization
} // namespace pmpc
