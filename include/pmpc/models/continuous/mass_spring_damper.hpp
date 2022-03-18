#pragma once

#include <pmpc/macros.hpp>
#include <pmpc/models/continuous/linear.hpp>

namespace pmpc {
namespace models {
namespace continuous {


/// Definition of a mass-spring-damper linear system.
/** This class represents a linear system in the form:
  * \f[
  *   a = \frac{u}{m} - \omega_n^2 p - 2 \xi \omega_n v
  * \f]
  * wherein:
  * - \f$ m \f$ is the mass of the system.
  * - \f$ \omega_n \f$ is the natural pulsation of the mass-spring system.
  * - \f$ \xi \f$ is the damping factor.
  * - \f$ u \f$ is a force applied on the mass and is the (scalar) input of
  *   the system.
  * - \f$ p, v=\dot{p} \f$ represent the position and velocity of the mass and
  *   also represent the state vector of the system.
  * - \f$ a=\ddot{p} \f$ is the acceleration of the mass.
  */
template<class Scalar>
class MassSpringDamper : public Linear<Scalar> {
public:
  // Allows the use of VectorXs and MatrixXs
  PMPC_MATRIX_VECTOR_TYPEDEFS(Scalar);

  using Linear<Scalar>::A;
  using Linear<Scalar>::B;

  /// Creates a MSD system from the given constants.
  /** The three inputs are all templetized to allow more flexibility.
    * @param m mass of the system.
    * @param wn natural pulsation \f$ \omega_n \f$.
    * @param xi damping factor \f$ \xi \f$.
    */
  template<class P1=Scalar,class P2=Scalar,class P3=Scalar>
  MassSpringDamper(
    const P1& m,
    const P2& wn,
    const P3& xi
  )
  : Linear<Scalar>(2,1)
  {
    A(0,0) = 0;
    A(0,1) = 1;
    A(1,0) = -wn*wn;
    A(1,1) = -2*xi*wn;
    B(1,0) = 1/m;
  }
};


} // continuous
} // models
} // pmpc
