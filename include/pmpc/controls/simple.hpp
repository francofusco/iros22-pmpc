#pragma once

#include <pmpc/macros.hpp>
#include <pmpc/controls/linear.hpp>

namespace pmpc {
namespace controls {

/// Simple parameterization corresponding to the "classical" one.
/** \todo Long description of the Simple parameterization.
  */
template<class Scalar>
class Simple : public Linear<Scalar> {
public:
/// \cond
  // Allows the use of VectorXs and MatrixXs
  PMPC_MATRIX_VECTOR_TYPEDEFS(Scalar);
/// \endcond

  /// Create a "classical" parameterization.
  /** @param dim_u dimension of the control vector of the system.
    * @param np prediction horizon.
    * @param nc control horizon (must be smaller than or equal to np).
    */
  Simple(const int& dim_u, const int& np, const int& nc);

  /// Create a "classical" parameterization.
  /** This version assumes that the control horizon equlas the prediction one.
    * @param dim_u dimension of the control vector of the system.
    * @param np prediction horizon.
    */
  Simple(const int& dim_u, const int& np);

  using Linear<Scalar>::dim_p;
  using Linear<Scalar>::dim_u;
  using Linear<Scalar>::np;

  const int nc; ///< Control horizon.

protected:
  using Linear<Scalar>::P;

  /// Specialization of Parameterization<Scalar>::controlSample_impl().
  virtual void controlSample_impl(
    const Eigen::Ref<const VectorXs> p,
    const int& k,
    Eigen::Ref<VectorXs> uk,
    Eigen::Ref<MatrixXs>* Pk
  ) const override;
};

} // namespace controls
} // namespace pmpc

