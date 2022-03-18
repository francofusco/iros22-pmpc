#pragma once

#include <pmpc/macros.hpp>
#include <pmpc/controls/parameterization.hpp>

namespace pmpc {
namespace controls {

/// Base class for linear parameterizations.
/** Linear parameterizations are a particular case in which the control sequence
  * can be written as a linear combination of the parameters, *i.e.*,
  * \f$ \underline{\bm{u}} = \boldsymbol{\Pi} \bm{p} \f$ for a given constant
  * combination matrix \f$ \boldsymbol{\Pi} \f$.
  *
  * \note
  * This base class should not be used directly, as it will fill
  * \f$ \boldsymbol{\Pi} \f$ with zeros. Use one of its children classes instead.
  */
template<class Scalar>
class Linear : public Parameterization<Scalar> {
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
  Linear(
    const int& dim_p,
    const int& dim_u,
    const int& np
  );

  using Parameterization<Scalar>::dim_p;
  using Parameterization<Scalar>::dim_u;
  using Parameterization<Scalar>::np;

  /// Returns the combination matrix of this linear parameterization.
  inline const MatrixXs& getP() const { return P; }

protected:
  /// Specialization of Parameterization<Scalar>::controlSample_impl().
  virtual void controlSample_impl(
    const Eigen::Ref<const VectorXs> p,
    const int& k,
    Eigen::Ref<VectorXs> uk,
    Eigen::Ref<MatrixXs>* Pk
  ) const override;

  /// Specialization of Parameterization<Scalar>::controlSequence_impl().
  virtual void controlSequence_impl(
    const Eigen::Ref<const VectorXs> p,
    Eigen::Ref<VectorXs> u,
    Eigen::Ref<MatrixXs>* P
  ) const;

  MatrixXs P; ///< Combination matrix.
};

} // namespace controls
} // namespace pmpc

