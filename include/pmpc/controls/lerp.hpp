#pragma once

#include <pmpc/macros.hpp>
#include <pmpc/controls/linear.hpp>
#include <vector>


namespace pmpc {
namespace controls {

/// Linear interpolation parameterization.
/** \todo Long description of the LERP parameterization.
  */
template<class Scalar>
class LERP : public Linear<Scalar> {
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
  LERP(
    const int& dim_u,
    const int& np,
    const std::vector<int>& free_samples
  );

  inline const std::vector<int>& getFrees() const { return frees; }
  inline const std::vector<std::pair<int,Scalar>>& getPredecessor() const { return predecessor; }
  inline const std::vector<std::pair<int,Scalar>>& getSuccessor() const { return predecessor; }

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
  std::vector<std::pair<int,Scalar>> predecessor; ///< Index and weight of a predecessor.
  std::vector<std::pair<int,Scalar>> successor; ///< Index and weight of a successor.
  std::vector<bool> is_free; ///< Tells if a sample is free.
  int last_free; ///< Index of the last free sample.
};

} // namespace controls
} // namespace pmpc

