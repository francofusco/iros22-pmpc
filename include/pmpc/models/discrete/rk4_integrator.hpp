#pragma once

#include <pmpc/macros.hpp>
#include <pmpc/models/discrete/model.hpp>
#include <pmpc/models/continuous/model.hpp>

namespace pmpc {
namespace models {
namespace discrete {

/// Discretization of a continuous time model.
/** This class allows to discretize a continuous-time model via a Runge-Kutta
  * explicit method of order 4. Given a continuous-time model,
  * this class generates the following discrete-time model:
  * \f[
  *   \bm{x}_{k+1} =
  *   \bm{x}_k +
  *   \frac{\Delta t}{6}
  *   \left( \bm{k}_1 + 2\bm{k}_2 + 2\bm{k}_3 + \bm{k}_4
  *   \right)
  * \f]
  * with \f$ \Delta t \f$ representing the sampling time-step and:
  * \f{align*}{
  *   \bm{k}_1 &= \bm{f}(\bm{x}_k, \bm{u}_k) \\
  *   \bm{k}_2 &= \bm{f}\left(\bm{x}_k + \frac{\Delta t}{2}\bm{k}_1, \bm{u}_k\right) \\
  *   \bm{k}_3 &= \bm{f}\left(\bm{x}_k + \frac{\Delta t}{2}\bm{k}_2, \bm{u}_k\right) \\
  *   \bm{k}_4 &= \bm{f}(\bm{x}_k + \Delta t\,\bm{k}_3, \bm{u}_k)
  * \f}
  *
  * The jacobians can be evaluated as:
  * \f[
  *   \frac{\partial\bm{x}_{k+1}}{\partial\bm{x}_k}
  *   =
  *   \mathbf{I} +
  *   \frac{\Delta t}{6}
  *   \left(
  *       \frac{\partial\bm{k}_1}{\partial\bm{x}_k}
  *       +
  *       2\frac{\partial\bm{k}_2}{\partial\bm{x}_k}
  *       +
  *       2 \frac{\partial\bm{k}_3}{\partial\bm{x}_k}
  *       +
  *       \frac{\partial\bm{k}_4}{\partial\bm{x}_k}
  *   \right)
  * \f]
  * \f[
  *   \frac{\partial\bm{x}_{k+1}}{\partial\bm{u}_k}
  *   =
  *   \frac{\Delta t}{6}
  *   \left(
  *       \frac{\partial\bm{k}_1}{\partial\bm{u}_k}
  *       +
  *       2\frac{\partial\bm{k}_2}{\partial\bm{u}_k}
  *       +
  *       2 \frac{\partial\bm{k}_3}{\partial\bm{u}_k}
  *       +
  *       \frac{\partial\bm{k}_4}{\partial\bm{u}_k}
  *   \right)
  * \f]
  * with:
  * \f{align*}{
  *       \frac{\partial\bm{k}_1}{\partial\bm{x}_k}
  *       &=
  *       \mathbf{F}_x(\bm{x}_k,\bm{u}_k)
  *       \\
  *       \frac{\partial\bm{k}_1}{\partial\bm{u}_k}
  *       &=
  *       \mathbf{F}_u(\bm{x}_k,\bm{u}_k)
  *       \\
  *       \frac{\partial\bm{k}_2}{\partial\bm{x}_k}
  *       &=
  *       \mathbf{F}_x\left(\bm{x}_k+\frac{\Delta t}{2}\bm{k}_1,\bm{u}_k\right)
  *       \left( \mathbf{I} + \frac{\Delta t}{2}\frac{\partial\bm{k}_1}{\partial\bm{x}_k} \right)
  *       \\
  *       \frac{\partial\bm{k}_2}{\partial\bm{u}_k}
  *       &=
  *       \mathbf{F}_u\left(\bm{x}_k+\frac{\Delta t}{2}\bm{k}_1,\bm{u}_k\right)
  *       +
  *       \mathbf{F}_x\left(\bm{x}_k+\frac{\Delta t}{2}\bm{k}_1,\bm{u}_k\right)
  *       \frac{\Delta t}{2}\frac{\partial\bm{k}_1}{\partial\bm{u}_k}
  *       \\
  *       \frac{\partial\bm{k}_3}{\partial\bm{x}_k}
  *       &=
  *       \mathbf{F}_x\left(\bm{x}_k+\frac{\Delta t}{2}\bm{k}_2,\bm{u}_k\right)
  *       \left( \mathbf{I} + \frac{\Delta t}{2}\frac{\partial\bm{k}_2}{\partial\bm{x}_k} \right)
  *       \\
  *       \frac{\partial\bm{k}_3}{\partial\bm{u}_k}
  *       &=
  *       \mathbf{F}_u\left(\bm{x}_k+\frac{\Delta t}{2}\bm{k}_2,\bm{u}_k\right)
  *       +
  *       \mathbf{F}_x\left(\bm{x}_k+\frac{\Delta t}{2}\bm{k}_2,\bm{u}_k\right)
  *       \frac{\Delta t}{2}\frac{\partial\bm{k}_2}{\partial\bm{u}_k}
  *       \\
  *       \frac{\partial\bm{k}_4}{\partial\bm{x}_k}
  *       &=
  *       \mathbf{F}_x\left(\bm{x}_k+\Delta t\,\bm{k}_3,\bm{u}_k\right)
  *       \left( \mathbf{I} + \Delta t\,\frac{\partial\bm{k}_2}{\partial\bm{x}_k} \right)
  *       \\
  *       \frac{\partial\bm{k}_4}{\partial\bm{u}_k}
  *       &=
  *       \mathbf{F}_u\left(\bm{x}_k+\Delta t\,\bm{k}_3,\bm{u}_k\right)
  *       +
  *       \mathbf{F}_x\left(\bm{x}_k+\Delta t\,\bm{k}_3,\bm{u}_k\right)
  *       \Delta t\,\frac{\partial\bm{k}_3}{\partial\bm{u}_k}
  * \f}
  */
template<class Scalar>
class RK4Integrator : public Model<Scalar> {
public:
  // Allows the use of VectorXs and MatrixXs
  PMPC_MATRIX_VECTOR_TYPEDEFS(Scalar);

  using Model<Scalar>::dim_x;
  using Model<Scalar>::dim_u;
  using Model<Scalar>::XZero;
  using Model<Scalar>::FXZero;
  using Model<Scalar>::FUZero;

  /// Initializes the model.
  /** @param dt sampling time used in the discretization.
    * @param model an instance of continuous::Model, representing the system
    *   to be discretized.
    */
  RK4Integrator(
    Scalar dt,
    const continuous::Model<Scalar>& model
  );

  /// Integrate a continuous-time model.
  /** This auxiliary method allows to perform integration of a given model.
    * Given an input state and a (constant) control sample, RK's integration
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

private:
  /// Auxiliary method to compute the coefficients for the integration.
  static void rk4coefficients(
    const continuous::Model<Scalar>& model,
    const Eigen::Ref<const VectorXs> x0,
    const Eigen::Ref<const VectorXs> u,
    Scalar T,
    Eigen::Ref<VectorXs> k1,
    Eigen::Ref<VectorXs> k2,
    Eigen::Ref<VectorXs> k3,
    Eigen::Ref<VectorXs> k4,
    MatrixXs* dk1dx=nullptr,
    MatrixXs* dk1du=nullptr,
    MatrixXs* dk2dx=nullptr,
    MatrixXs* dk2du=nullptr,
    MatrixXs* dk3dx=nullptr,
    MatrixXs* dk3du=nullptr,
    MatrixXs* dk4dx=nullptr,
    MatrixXs* dk4du=nullptr
  );

  // Auxiliary structures used in function_impl. They are here simply
  // to avoid allocating the memory every time the method is called.
  mutable VectorXs k1, k2, k3, k4;
  mutable MatrixXs dk1dx, dk1du;
  mutable MatrixXs dk2dx, dk2du;
  mutable MatrixXs dk3dx, dk3du;
  mutable MatrixXs dk4dx, dk4du;
};

} // discrete
} // models
} // pmpc

