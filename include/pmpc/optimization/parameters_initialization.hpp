#pragma once

#ifdef PMPC_WITH_QPOASES
  #include <qpOASES.hpp>
#endif

#include <pmpc/exceptions.hpp>
#include <pmpc/controls/linear.hpp>
#include <pmpc/optimization/linear_control_constraints.hpp>


namespace pmpc {
namespace optimization {

/// Initializes the parameters to fit (in least square sense) a given control sequence.
/** Given a linear parameterization with combination matrix \f$\bm{\Pi}\f$ and
  * a desired control sequence \f$\underline{\bm{u}}^\star\f$, this function
  * tries to solve the following linear regression problem:
  * \f[
  *   \min_{\bm{p}} \lVert \bm{\Pi}\bm{p}-\underline{\bm{u}}^\star \rVert^2
  * \f]
  * @param[in] param a linear control parameterization.
  * @param[in] useq desired control sequence. It should be a vector of length
  *   either `param.dim_u*param.np` or `param.dim_u`. In the latter case, the
  *   vector is considered to be a single control sample, and is thus repeated
  *   `param.np` times to define the whole control sequence. A DimensionError is
  *   thrown if the dimensions are incorrect.
  * @param[out] p parameter vector that best fit the given control sequence.
  */
template<class Scalar>
void initializeParameters(
  const controls::Linear<Scalar>& param,
  const Eigen::Ref<const Eigen::Matrix<Scalar,Eigen::Dynamic,1>> useq,
  Eigen::Ref<Eigen::Matrix<Scalar,Eigen::Dynamic,1>> p
);

/// Initializes the parameters to fit (in least square sense) a given control sequence.
/** Given a linear parameterization with combination matrix \f$\bm{\Pi}\f$ and
  * a desired control sequence \f$\underline{\bm{u}}^\star\f$, this function
  * tries to solve the following constrained linear regression problem:
  * \f[
  *   \min_{\bm{p}} \lVert \bm{\Pi}\bm{p}-\underline{\bm{u}}^\star \rVert^2
  *   \quad \text{s.t.} \; \mathrm{C}\bm{p} \leq \bm{d}
  * \f]
  * @param[in] constraints a set of linear constraints to be enforced. This
  *   instance will also be used to obtain the control parameterization.
  * @param[in] useq desired control sequence. It should be a vector of length
  *   either `param.dim_u*param.np` or `param.dim_u`. In the latter case, the
  *   vector is considered to be a single control sample, and is thus repeated
  *   `param.np` times to define the whole control sequence. A DimensionError is
  *   thrown if the dimensions are incorrect.
  * @param[out] p parameter vector that best fit the given control sequence.
  * \warning This function is available only if qpOASES is installed. Otherwise,
  * a NotImplemented exception will be thrown. In addition, since qpOASES
  * relies on a specific `real_t` datatype, the template `Scalar` must match
  * `REFER_NAMESPACE_QPOASES real_t` (should be `double`). If not, a BadType
  * exception will be thrown.
  */
template<class Scalar>
void initializeParameters(
  LinearControlConstraints<Scalar>& constraints,
  const Eigen::Ref<const Eigen::Matrix<Scalar,Eigen::Dynamic,1>> useq,
  Eigen::Ref<Eigen::Matrix<Scalar,Eigen::Dynamic,1>> p
);

} // namespace optimization
} // namespace pmpc

