#pragma once

#include <pmpc/macros.hpp>
#include <pmpc/predictors/recursive.hpp>
#include <pmpc/controls/linear.hpp>
#include <pmpc/models/discrete/linear.hpp>


namespace pmpc {
namespace predictors {

/// Class that allows to perform predictions for linear models with linear parameterizations.
/** When dealing with linear models and linear parameterizations, the prediction
  * vector \f$\underline{\bm{x}}\f$ can be written as a linear-affine
  * function of the initial state \f$\bm{x}_0\f$ and the parameters
  * \f$\bm{p}\f$:
  * \f[
  *   \underline{\bm{x}}
  *     = \boldsymbol{\mu} + \tilde{\mathbf{B}}\bm{p}
  *     = \tilde{\mathbf{A}}\bm{x}_0 + \tilde{\bm{w}} + \tilde{\mathbf{B}}\bm{p}
  * \f]
  */
template<class Scalar>
class Linear : public Recursive<Scalar> {
public:
/// \cond
  // Allows the use of VectorXs and MatrixXs
  PMPC_MATRIX_VECTOR_TYPEDEFS(Scalar);
/// \endcond

  /// Initializes the predictor.
  /** This constructor initializes the matrices of the predction assuming
    * a null initial state.
    * @param linear_parameterization linear control parameterization used along
    *   the prediction horizon.
    * @param linear_model linear discrete-time model for which predictions have
    *   to be computed.
    */
  Linear(
    const controls::Linear<Scalar>& linear_parameterization,
    const models::discrete::Linear<Scalar>& linear_model
  );

  /// Initializes the predictor.
  /** This constructor initializes the matrices of the predction from the given
    * initial state.
    * @param linear_parameterization linear control parameterization used along
    *   the prediction horizon.
    * @param linear_model linear discrete-time model for which predictions have
    *   to be computed.
    * @param x0 initial configuration of the system at which all matrices can
    *   be initialized.
    */
  Linear(
    const controls::Linear<Scalar>& linear_parameterization,
    const models::discrete::Linear<Scalar>& linear_model,
    const Eigen::Ref<const VectorXs> x0
  );

  /// Compute the matrices of the linear prediction model.
  /** This method computes the matrices \f$\tilde{\mathbf{A}}\f$ and
    * \f$\tilde{\mathbf{B}}\f$, as well as the vector \f$\tilde{\bm{w}}\f$.
    *
    * \note As \f$\boldsymbol{\mu}\f$ depends also on \f$\bm{x}_0\f$, it is not
    * updated here. To to make this class recompute it, you should use
    * update(const Eigen::Ref<const VectorXs> x0) instead.
    */
  void update();

  /// Compute matrices of the linear prediction model.
  /** Similarly to update(), this method is used to update the matrices
    * appearing in the linear-affine prediction model. However, also
    * \f$\boldsymbol{\mu}\f$ is updated here.
    * @param x0 initial configuration from which predictions have to be
    *   computed for the linear model.
    * @param full_update if true, this method will first call update() to make
    *   sure that \f$\tilde{\mathbf{A}}\f$, \f$\tilde{\mathbf{B}}\f$, and
    *   \f$\tilde{\bm{w}}\f$ are up-to-date. If false, it is assumed that these
    *   quantities have been already computed before, and thus only
    *   \f$\boldsymbol{\mu}\f$ is re-computed.
    */
  void update(const Eigen::Ref<const VectorXs> x0, bool full_update);

  /// Returns the vector \f$\boldsymbol{\mu}\f$.
  inline const VectorXs& getMu() const { return mu; }
  /// Returns the matrix \f$\tilde{\mathbf{A}}\f$.
  inline const MatrixXs& getAt() const { return At; }
  /// Returns the vector \f$\tilde{\bm{w}}\f$.
  inline const VectorXs& getWt() const { return wt; }
  /// Returns the matrix \f$\tilde{\mathbf{B}}\f$.
  inline const MatrixXs& getBt() const { return Bt; }

  const controls::Linear<Scalar>& linparam; ///< Linear control parameterization.
  const models::discrete::Linear<Scalar>& linmodel; ///< Linear model for which predictions are performed.

private:
  VectorXs mu; ///< Vector \f$\boldsymbol{\mu}\f$
  MatrixXs At; ///< Matrix \f$\tilde{\mathbf{A}}\f$
  VectorXs wt; ///< Vector \f$\tilde{\bm{w}}\f$
  MatrixXs Bt; ///< Matrix \f$\tilde{\mathbf{B}}\f$
};

} // namespace predictors
} // namespace pmpc

