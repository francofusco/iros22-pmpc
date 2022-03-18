#pragma once

#include <pmpc/macros.hpp>
#include <pmpc/exceptions.hpp>

namespace pmpc {
namespace controls {

/// Abstract base class of all parameterizations.
template<class Scalar>
class Parameterization {
public:
/// \cond
  // Allows the use of VectorXs and MatrixXs
  PMPC_MATRIX_VECTOR_TYPEDEFS(Scalar);
/// \endcond

  /// Store information about dimensions.
  /** @param dim_p dimension of the parameter vector.
    * @param dim_u dimension of the control vector of the system.
    * @param np prediction horizon.
    */
  Parameterization(const int& dim_p, const int& dim_u, const int& np);

  /// Virtual destructor, added as a best practice.
  virtual ~Parameterization() = default;

  const int dim_p; ///< Parameter dimension.
  const int dim_u; ///< Input dimension.
  const int np; ///< Prediction horizon.

  /// Convenience method that returns a zero-valued parameter vector.
  inline VectorXs PZero() const { return VectorXs::Zero(dim_p); }
  /// Convenience method that returns a zero-valued control sample.
  inline VectorXs UZero() const { return VectorXs::Zero(dim_u); }
  /// Convenience method that returns a zero-valued control sequence.
  inline VectorXs UUZero() const { return VectorXs::Zero(dim_u*np); }

  /// Evaluate the k-th control sample from the given parameters.
  /** @param[in] p vector of free-parameters.
    * @param[in] k sample index.
    * @param[out] uk control sample
    *   \f$ \bm{u}_k = \boldsymbol{\pi}_k(\bm{p}) \f$.
    */
  inline void controlSample(
    const Eigen::Ref<const VectorXs> p,
    const int& k,
    Eigen::Ref<VectorXs> uk
  ) const
  {
    controlSample_impl(p, k, uk, nullptr);
  }

  /// Evaluate the k-th control sample and its jacobian from the given parameters.
  /** @param[in] p vector of free-parameters.
    * @param[in] k sample index.
    * @param[out] uk control sample
    *   \f$ \bm{u}_k = \boldsymbol{\pi}_k(\bm{p}) \f$.
    * @param[out] Pk jacobian of the control sample, *i.e.*,
    *   \f$ \frac{\bm{u}_k}{\bm{p}} \f$.
    */
  inline void controlSample(
    const Eigen::Ref<const VectorXs> p,
    const int& k,
    Eigen::Ref<VectorXs> uk,
    Eigen::Ref<MatrixXs> Pk
  ) const
  {
    controlSample_impl(p, k, uk, &Pk);
  }

  /// Evaluate the whole control sequence given the parameters.
  /** @param p vector of free-parameters.
    * @param[out] u whole control sequence, obtained by stacking all control
    *   samples on top of each other.
    */
  inline void controlSequence(
    const Eigen::Ref<const VectorXs> p,
    Eigen::Ref<VectorXs> u
  ) const
  {
    controlSequence_impl(p, u, nullptr);
  }

  /// Evaluate the whole control sequence and its jacobian given the parameters.
  /** @param p vector of free-parameters.
    * @param[out] u whole control sequence, obtained by stacking all control
    *   samples on top of each other.
    * @param[out] P whole control sequence jacobian, obtained by stacking the
    *   individual sample jacobians on top of each other.
    */
  inline void controlSequence(
    const Eigen::Ref<const VectorXs> p,
    Eigen::Ref<VectorXs> u,
    Eigen::Ref<MatrixXs> P
  ) const
  {
    controlSequence_impl(p, u, &P);
  }

  /** \details \todo This method should be useful to perform "sparsity-aware"
    * matrix multiplication. It could exploit some knowledge on the structure
    * of the (possibly) sparse control Jacobian. However, I am not yet sure
    * how this could be addressed (and if it actually improve performances...)
    */
  virtual void sparseJacobianMultiplication(
    const Eigen::Ref<const MatrixXs> M,
    const Eigen::Ref<const MatrixXs> Pk,
    Eigen::Ref<MatrixXs> R
  ) const
  {
    R = M * Pk;
  }

protected:
  /// Evaluate the k-th control sample from the given parameters.
  /** This method must be overridden in sub-classes to provide specific
    * parameterizations.
    * @param[in] p vector of free-parameters.
    * @param[in] k sample index.
    * @param[out] uk control sample
    *   \f$ \bm{u}_k = \boldsymbol{\pi}_k(\bm{p}) \f$.
    * @param[out] Pk jacobian of the k-th control sample, *i.e.*,
    *   \f$ \frac{\bm{u}_k}{\bm{p}} \f$. If nullptr is passed, then the
    *   jacobian should not be computed inside the function call.
    */
  virtual void controlSample_impl(
    const Eigen::Ref<const VectorXs> p,
    const int& k,
    Eigen::Ref<VectorXs> uk,
    Eigen::Ref<MatrixXs>* Pk
  ) const = 0;

  /// Evaluate the whole control sequence from the given parameters.
  /** By default, this method calls controlSample_impl() repeatedly, thus
    * stacking all control samples and jacobians.
    * However, it can be overridden in sub-classes if this provides a
    * performance boost.
    * @param[in] p vector of free-parameters.
    * @param[out] u stacked sequence of all control samples along the prediction
    *   horizon.
    * @param[out] P jacobian of the whole control sequence with respect to the
    *   parameters. If nullptr is passed, then the jacobian is not computed.
    */
  virtual void controlSequence_impl(
    const Eigen::Ref<const VectorXs> p,
    Eigen::Ref<VectorXs> u,
    Eigen::Ref<MatrixXs>* P
  ) const;

};

} // namespace controls
} // namespace pmpc

