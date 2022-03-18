#pragma once

#include <pmpc/macros.hpp>
#include <pmpc/controls/linear.hpp>
#include <vector>


namespace pmpc {
namespace controls {

/// Zero-Order-Holder parameterization.
/** \todo Long description of the ZOH parameterization.
  */
template<class Scalar>
class ZOH : public Linear<Scalar> {
public:
/// \cond
  // Allows the use of VectorXs and MatrixXs
  PMPC_MATRIX_VECTOR_TYPEDEFS(Scalar);
/// \endcond

  /// Create a "classical" parameterization.
  /** @param dim_u dimension of the control vector of the system.
    * @param np prediction horizon.
    * @param free_samples indices of the samples whose value can be freely
    *   assigned by the MPC scheme. It cannot contain negative values, or
    *   values equal to or greater than np. Some processing should be performed
    *   internally, to make sure that it is sorted in ascending order and that
    *   it starts with a zero.
    */
  ZOH(
    const int& dim_u,
    const int& np,
    std::vector<int> free_samples
  );

  using Linear<Scalar>::dim_p;
  using Linear<Scalar>::dim_u;
  using Linear<Scalar>::np;

protected:
  using Linear<Scalar>::P;

  /// Specialization of Parameterization<Scalar>::controlSample_impl().
  virtual void controlSample_impl(
    const Eigen::Ref<const VectorXs> p,
    const int& k,
    Eigen::Ref<VectorXs> uk,
    Eigen::Ref<MatrixXs>* Pk
  ) const override;

private:
  std::vector<int> frees; ///< Indices of the free samples.
  std::vector<int> predecessor; ///< Indices of predecessors of a specific sample.
};

} // namespace controls
} // namespace pmpc

