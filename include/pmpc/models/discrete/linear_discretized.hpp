#pragma once

#include <pmpc/macros.hpp>
#include <pmpc/models/discrete/linear.hpp>
#include <pmpc/models/continuous/linear.hpp>

namespace pmpc {
namespace models {
namespace discrete {

/// Discretization of a linear affine system using matrix exponentiation.
/** This class allows to obtain the discretized version of a continuous::Linear
  * model. It uses matrix exponentiation under the assumption of
  * Zero-Order-Holded input.
  *
  * Given the continuous time model
  * \f[
  *   \dot{\bm{x}} = \mathbf{A} \bm{x} + \mathbf{B} \bm{u} + \bm{w}
  * \f]
  * The discretized model writes as:
  * \f[
  *   \bm{x}_{k+1} = \mathbf{A}_d \bm{x}_k + \mathbf{B}_d \bm{u}_k + \bm{w}_d
  * \f]
  * where the constant terms are obtained from:
  * \f[
  *   \begin{bmatrix}
  *     \mathbf{A}_d & \mathbf{B}_d & \mathbf{W}_d \\
  *     \mathbf{0} & \mathbf{I} & \mathbf{0} \\
  *     \mathbf{0} & \mathbf{0} & \mathbf{I}
  *   \end{bmatrix}
  *   =
  *   \exp \left(
  *     \Delta t\,
  *     \begin{bmatrix}
  *       \mathbf{A} & \mathbf{B} & \mathbf{I} \\
  *       \mathbf{0} & \mathbf{0} & \mathbf{0}
  *    \end{bmatrix}
  *    \right)
  *    \qquad \bm{w}_d = \mathbf{W}_d\bm{w}
  * \f]
  */
template<class Scalar>
class LinearDiscretized : public Linear<Scalar> {
public:
  // Allows the use of VectorXs and MatrixXs
  PMPC_MATRIX_VECTOR_TYPEDEFS(Scalar);

  /// Constructor.
  /** Creates the discretized model given the sampling time and the continuous
    * time model.
    * @param dt time-step used in the discretization.
    * @param model an instance of continuous::Linear representing the model to
    *   be discretized.
    */
  LinearDiscretized(
    Scalar dt,
    const continuous::Linear<Scalar>& model
  );

  /// Recompute the discretization of the system.
  /** This function recomputes the discretized matrices of the linear system.
    * If the original system is time-invariant, then this method does not
    * need to be called by the user. However, the original linear system might
    * actually come from the linearization of a generic model. If this is the
    * case, it is possible to update the linearization and then to recompute
    * the discretized system via this function.
    */
  void update();

  /// Discretization of a continuous::Linear model.
  /** This auxiliary method allows to perform matrix exponentiation to obtain
    * the discretized model of a continuous time linear system.
    * @param[in] model the linear model to be discretized.
    * @param[in] dt sampling time for the discretization.
    * @param[out] Ad discretized state matrix \f$ \mathbf{A}_d \f$.
    * @param[out] Bd discretized control matrix \f$ \mathbf{B}_d \f$.
    * @param[out] Wd discretized affine vector \f$ \bm{w}_d \f$.
    */
  static void discretize(
    const continuous::Linear<Scalar>& model,
    Scalar dt,
    Eigen::Ref<MatrixXs> Ad,
    Eigen::Ref<MatrixXs> Bd,
    Eigen::Ref<VectorXs> Wd
  );

  const continuous::Linear<Scalar>& c_model; ///< Continuous time, linear system to be discretized.
  const Scalar dt; ///< Sampling time used in the discretization.

protected:
  using Linear<Scalar>::A;
  using Linear<Scalar>::B;
  using Linear<Scalar>::W;
};

} // discrete
} // models
} // pmpc

