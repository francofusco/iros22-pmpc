#pragma once

#include <pmpc/macros.hpp>
#include <pmpc/controls/parameterization.hpp>
#include <pmpc/models/discrete/model.hpp>

namespace pmpc {
namespace predictors {

/// Class that allows to obtain a control sequence over the prediction horizon.
/** \todo Detailed description.
  */
template<class Scalar>
class Recursive {
public:
/// \cond
  // Allows the use of VectorXs and MatrixXs
  PMPC_MATRIX_VECTOR_TYPEDEFS(Scalar);
/// \endcond

  /// Creates the predictor given the parameterization and the model.
  /** @param param the control input parameterization used to control the system.
    * @param model a discrete-time models for which predictions are to be
    *   evaluated.
    */
  Recursive(
    const controls::Parameterization<Scalar>& param,
    const models::discrete::Model<Scalar>& model
  );

  /// Evaluate the prediction of the states over the prediction horizon.
  inline void prediction(
    const Eigen::Ref<const VectorXs> x0,
    const Eigen::Ref<const VectorXs> p,
    Eigen::Ref<VectorXs> xx
  ) const
  {
    prediction_impl(x0, p, xx, nullptr);
  }

  /// Evaluate the prediction of the states and its jacobian over the prediction horizon.
  inline void prediction(
    const Eigen::Ref<const VectorXs> x0,
    const Eigen::Ref<const VectorXs> p,
    Eigen::Ref<VectorXs> xx,
    Eigen::Ref<MatrixXs> J
  ) const
  {
    prediction_impl(x0, p, xx, &J);
  }

  const controls::Parameterization<Scalar>& param; ///< Reference to the control parameterization used in the prediction.
  const models::discrete::Model<Scalar>& model; ///< Reference to the model for which predictions are evaluated.

protected:
  /// Low-level method that evaluates the predictions.
  /** This method successively evaluate future states by applying the current
    * control sample to the last predicted state sample. The method can be
    * overridden in case a faster implementaion is possible.
    * @param[in] x0 initial state of the model.
    * @param[in] p vector of free-parameters that allow to evaluate the controls
    *   to be applied to the system.
    * @param[out] xx prediction vector (containing all predicted state samples).
    * @param[out] J jacobian of the prediction vector with respect to the
    *   parameters. If nullptr is passed, then the jacobian should not be
    *   computed in the method call.
    */
  virtual void prediction_impl(
    const Eigen::Ref<const VectorXs> x0,
    const Eigen::Ref<const VectorXs> p,
    Eigen::Ref<VectorXs> xx,
    Eigen::Ref<MatrixXs>* J
  ) const;

  // Auxiliary matrices used in the calculations of the prediction vector and
  // of its jacobian. They are here to avoid allocating the memory every time.
  mutable MatrixXs Fx, Fu, Pk;
  mutable VectorXs uk;
};

} // namespace predictors
} // namespace pmpc

