#pragma once

#include <pmpc/macros.hpp>
#include <pmpc/controls/linear.hpp>
#include <functional>

namespace pmpc {
namespace controls {

/// Parameterization based on the concept of basis functions.
/** \todo Long description of this parameterization.
  */
template<class Scalar>
class FunctionBasis : public Linear<Scalar> {
public:
/// \cond
  // Allows the use of VectorXs and MatrixXs
  PMPC_MATRIX_VECTOR_TYPEDEFS(Scalar);
/// \endcond

  /// Initializes the linear parameter matrix.
  /** @param dim_u dimension of the control vector of the system.
    * @param np prediction horizon.
    * @param nb number of basis functions.
    * @param offset if true, the first basis function will be a constant offset
    *   and the remaining nb-1 functions will be the functions drawn from the
    *   provided basis.
    * @param dt sampling time.
    * @param func callable object that is able to sample basis functions. Its
    *   signature should be in the form `Scalar func(Scalar t, int i)`, meaning
    *   that it will return the value of the `i`-th basis function at the time
    *   instant `t`.
    * @param deriv callable object that is able to sample the derivatives of
    *   the basis functions.
    */
  FunctionBasis(
    const int& dim_u,
    const int& np,
    const int& nb,
    bool offset,
    Scalar dt,
    std::function<Scalar(Scalar,int)> func,
    std::function<Scalar(Scalar,int)> deriv
  );

  /// Returns the combination matrix of the derivatives of the basis functions.
  inline const MatrixXs& getPd() const { return Pd; }

  using Linear<Scalar>::dim_p;
  using Linear<Scalar>::dim_u;
  using Linear<Scalar>::np;

  const Scalar dt; ///< Sampling time.

  static FunctionBasis<Scalar> Laguerre(
    const int& dim_u,
    const int& np,
    const int& nb,
    bool offset,
    Scalar dt,
    Scalar tau
  );

  static FunctionBasis<Scalar>* LaguerrePtr(
    const int& dim_u,
    const int& np,
    const int& nb,
    bool offset,
    Scalar dt,
    Scalar tau
  );

  /// Damped polynomials basis.
  /** \f[
    *   \phi_i(t) = \left(\frac{t}{i\tau}\right)^i e^{i-t/\tau}
    * \f]
    * \f[
    *   \dot{\phi}_i(t) = \frac{1}{\tau}\left(\frac{t}{i\tau}\right)^{i-1} e^{i-t/\tau} \left(1 - \frac{t}{i\tau}\right)
    * \f]
    * \f[
    *   \ddot{\phi}_i(t) = \frac{1}{\tau^2}\left(\frac{t}{i\tau}\right)^{i-2} e^{i-t/\tau} \left(\frac{i-1}{i} - 2 \frac{t}{i\tau} + \left(\frac{t}{i\tau}\right)^2\right)
    * \f]
    * Maxima of \f$\phi_i\f$ are located at \f$t = i\tau\f$, while steepest points
    * are at \f$t = \tau\left(i\pm\sqrt{i}\right)\f$
    */
  static FunctionBasis<Scalar> DampedPoly(
    const int& dim_u,
    const int& np,
    const int& nb,
    bool offset,
    Scalar dt,
    Scalar tau
  );

  static FunctionBasis<Scalar>* DampedPolyPtr(
    const int& dim_u,
    const int& np,
    const int& nb,
    bool offset,
    Scalar dt,
    Scalar tau
  );

protected:
  using Linear<Scalar>::P;
  MatrixXs Pd; ///< Combination matrix for the derivative of the basis functions.

private:
  static Scalar laguerre_func(Scalar t, int i, Scalar tau);
  static Scalar laguerre_deriv(Scalar t, int i, Scalar tau);
  static Scalar dampedpoly_func(Scalar t, int i, Scalar tau);
  static Scalar dampedpoly_deriv(Scalar t, int i, Scalar tau);
};

} // namespace controls
} // namespace pmpc

