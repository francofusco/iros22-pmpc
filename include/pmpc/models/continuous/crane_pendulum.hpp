#pragma once

#include <pmpc/macros.hpp>
#include <pmpc/models/continuous/model.hpp>

namespace pmpc {
namespace models {
namespace continuous {

/// Underactuated pendulum with sliding base.
/** \todo Long description of CranePendulum. Remember to include some data.
  */
template<class Scalar>
class CranePendulum : public Model<Scalar> {
public:
/// \cond
  // Allows the use of VectorXs and MatrixXs
  PMPC_MATRIX_VECTOR_TYPEDEFS(Scalar);
/// \endcond

  /// Constructor that creates a new CranePendulum instance.
  /** @param support_mass mass of the moving base.
    * @param disk_mass mass of the pendulum.
    * @param disk_inertia inertia of the pendulum (with respect to its own
    *   center of mass).
    * @param rod_length distance between the revolute joint and the center of
    *   mass of the pendulum.
    * @param angle_offset angle (with respect to the downward direction) at
    *   which the pendulum should be considered at "zero position". As an
    *   example, if in your convention the pendulum is at the "zero" when it is
    *   pointing upward, then you should pass \f$\pi\f$ as value.
    */
  CranePendulum(
    Scalar support_mass,
    Scalar disk_mass,
    Scalar disk_inertia,
    Scalar rod_length,
    Scalar angle_offset=0
  );

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
  Scalar M; ///< Mass of the moving base.
  Scalar m; ///< Mass of the weight.
  Scalar I; ///< Inertia of the weight.
  Scalar ell; ///< Center of mass of the weight (from the revolute joint).
  Scalar offset; ///< Offset angle with respect to the downward position.
  Scalar mt; ///< Total mass.
  Scalar I0; ///< Equivalent inertia of the weight at the revolute joint.
  Scalar ml; ///< Disk mass times `ell`.
  Scalar ml2; ///< Square of `ml`.
  Scalar ml3; ///< Cube of `ml`.
  Scalar g; ///< Gravity acceleration.
};

} // continuous
} // models
} // pmpc

