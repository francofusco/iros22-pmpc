#pragma once

#include <pmpc/macros.hpp>
#include <pmpc/models/discrete/model.hpp>
#include <pmpc/models/continuous/model.hpp>

namespace pmpc {
namespace models {
namespace discrete {

/// Discretization of a continuous time model.
/** This class allows to discretize a continuous-time model via Euler's
  * forward integration method. Given a continuous-time state-space model,
  * this class generates the following discrete-time model:
  * \f[
  *   \bm{x}_{k+1} =
  *   \bm{x}_k +
  *   \Delta t\, \bm{f}(\bm{x}_k, \bm{u}_k)
  * \f]
  * with \f$ \Delta t \f$ representing the sampling time-step.
  *
  * The jacobians of the discretized model thus write as:
  * \f{align*}{
  *   \frac{\partial\bm{x}_{k+1}}{\partial\bm{x}_k}
  *   &=
  *   \Delta t\,\frac{\partial\bm{f}}{\partial\bm{x}_k}
  *   +
  *   \bm{I}
  *   \\
  *   \frac{\partial\bm{x}_{k+1}}{\partial\bm{u}_k}
  *   &=
  *   \Delta t\,\frac{\partial\bm{f}}{\partial\bm{u}_k}
  * \f}
  */
template<class Scalar>
class EulerIntegrator : public Model<Scalar> {
public:
  // Allows the use of VectorXs and MatrixXs
  PMPC_MATRIX_VECTOR_TYPEDEFS(Scalar);

  using Model<Scalar>::dim_x;
  using Model<Scalar>::dim_u;

  /// Initializes the model.
  /** @param dt sampling time used in the discretization.
    * @param model an instance of continuous::Model, representing the system
    *   to be discretized.
    */
  EulerIntegrator(
    Scalar dt,
    const continuous::Model<Scalar>& model
  );

  /// Integrate a continuous-time model.
  /** This auxiliary method allows to perform integration of a given model.
    * Given an input state and a (constant) control sample, Euler's integration
    * is applied (possibly multiple times) to evaluate a new state sample
    * reached by the system.
    * @param[in] model the continuous time system to be integrated.
    * @param[in] x0 initial state sample.
    * @param[in] u control to be applied.
    * @param[out] x1 new state reached by the system.
    * @param[in] T time period during which the control is applied to the system.
    * @param[in] Nsteps number of euler steps to be performed. In practice, Nsteps
    *   will be iteratively applied by applying the control u for a period
    *   `T/Nsteps`.
    */
  static void integrate(
    const continuous::Model<Scalar>& model,
    const Eigen::Ref<const VectorXs> x0,
    const Eigen::Ref<const VectorXs> u,
    Eigen::Ref<VectorXs> x1,
    Scalar T,
    unsigned int Nsteps=1
  );

  const Scalar dt; ///< Discretization time step.
  const continuous::Model<Scalar>& c_model; ///< Reference to the continuous time model to be discretized.

protected:
  /// Specialization of Model<Scalar>::function_impl().
  virtual void function_impl(
    const Eigen::Ref<const VectorXs> x,
    const Eigen::Ref<const VectorXs> u,
    Eigen::Ref<VectorXs> f,
    Eigen::Ref<MatrixXs>* Fx,
    Eigen::Ref<MatrixXs>* Fu
  ) const override;

};

} // discrete
} // models
} // pmpc

