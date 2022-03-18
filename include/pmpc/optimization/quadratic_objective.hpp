#pragma once

#include <pmpc/macros.hpp>
#include <pmpc/optimization/objective_base.hpp>
#include <vector>

namespace pmpc {
namespace optimization {

/// Canonical objective that can be used in most MPC schemes.
/** This class represents a simple objective function for the MPC optimization
  * in which two terms concurrently try to minimize a state error and push
  * the control inputs toward a given "feedforward" value. In particular,
  * it takes the form:
  * \f[
  *   f_{obj}(\bm{p}) = \sum_{k=1}^{n_p} (\bm{x}_k-\bm{x}_k^\star)^T \mathbf{D}_{x,k} (\bm{x}_k-\bm{x}_k^\star)
  *   + \sum_{k=0}^{n_p-1} (\bm{u}_k-\bm{u}_k^\star)^T \mathbf{D}_{u,k} (\bm{u}_k-\bm{u}_k^\star)
  * \f]
  * The two matrices \f$\mathbf{D}_{x,k}\f$ and \f$\mathbf{D}_{u,k}\f$ are diagonal
  * matrices with non-negative entries and represent the weights of the different
  * contributions.
  *
  * An equivalent formulation is the following:
  * \f[
  *   f_{obj}(\bm{p}) =
  *     \lVert \underline{\bm{x}} - \underline{\bm{x}}^\star \rVert^2_{\mathbf{D}_x}
  *     +
  *     \lVert \underline{\bm{u}} - \underline{\bm{u}}^\star \rVert^2_{\mathbf{D}_u}
  *   = \left(\underline{\bm{x}} - \underline{\bm{x}}^\star\right)^T \mathbf{D}_x \left(\underline{\bm{x}} - \underline{\bm{x}}^\star\right)
  *     +
  *     \left(\underline{\bm{u}} - \underline{\bm{u}}^\star\right)^T \mathbf{D}_u \left(\underline{\bm{u}} - \underline{\bm{u}}^\star\right)
  * \f]
  * whose jacobian writes as:
  * \f[
  *   \frac{\partial f_{obj}}{\partial\bm{p}} =
  *     2 \left(\underline{\bm{x}} - \underline{\bm{x}}^\star\right)^T \mathbf{D}_x \frac{\partial\underline{\bm{x}}}{\partial\bm{p}}
  *     +
  *     2 \left(\underline{\bm{u}} - \underline{\bm{u}}^\star\right)^T \mathbf{D}_u \frac{\partial\underline{\bm{u}}}{\partial\bm{p}}
  * \f]
  *
  * Furthermore, this objective type can be approximated as a quadratic form
  * (Gauss-Newton approximation) by linearizing the terms
  * \f$ \underline{\bm{x}} \f$ and \f$ \underline{\bm{u}} \f$ inside the
  * objective. This leads to:
  * \f[
  *   f_{obj}(\bm{p}+\bm{\delta}_p) \simeq f_{gn}(\bm{\delta}_p) =
  *     \lVert \frac{\partial\underline{\bm{x}}}{\partial\bm{p}}\bm{\delta}_p + \underline{\bm{x}} - \underline{\bm{x}}^\star \rVert^2_{\mathbf{D}_x}
  *     +
  *     \lVert \frac{\partial\underline{\bm{u}}}{\partial\bm{p}}\bm{\delta}_p + \underline{\bm{u}} - \underline{\bm{u}}^\star \rVert^2_{\mathbf{D}_u}
  *     =
  *     \frac{1}{2}
  *     \bm{\delta}_p^T
  *     \underbrace{\left(
  *       2
  *       \frac{\partial\underline{\bm{x}}}{\partial\bm{p}}^T
  *       \mathbf{D}_x
  *       \frac{\partial\underline{\bm{x}}}{\partial\bm{p}}
  *       +
  *       2
  *       \frac{\partial\underline{\bm{u}}}{\partial\bm{p}}^T
  *       \mathbf{D}_u
  *       \frac{\partial\underline{\bm{u}}}{\partial\bm{p}}
  *     \right)}_{\mathbf{H}_{gn}}
  *     \bm{\delta}_p
  *     +
  *     \frac{\partial f_{obj}}{\partial\bm{p}} \bm{\delta}_p
  *     +
  *     f_{obj}
  * \f]
  *
  * \note
  * Concerning the actual implementation of the concepts above, it is easier
  * to keep the objective in the form:
  * \f[
  *   f_{obj}(\bm{p})
  *   =
  *   \sum_i \alpha_i \left(\underline{\bm{x}}_i - \underline{\bm{x}}_i^\star \right)^2
  *   +
  *   \sum_i \beta_i \left(\underline{\bm{u}}_i - \underline{\bm{u}}_i^\star \right)^2
  * \f]
  * wherein \f$ \underline{\bm{x}}_i \f$ and \f$ \underline{\bm{u}}_i \f$
  * represent the i-th element of the prediction vector and of the control
  * sequence. Indeed, in this way one can write the gradient of the objective as:
  * \f[
  *   \frac{\partial f_{obj}}{\partial\bm{p}}
  *   =
  *   \sum_i 2 \alpha_i \left(\underline{\bm{x}}_i - \underline{\bm{x}}_i^\star \right) \frac{\partial\underline{\bm{x}}_i}{\partial\bm{p}}
  *   +
  *   \sum_i 2 \beta_i \left(\underline{\bm{u}}_i - \underline{\bm{u}}_i^\star \right) \frac{\partial\underline{\bm{u}}_i}{\partial\bm{p}}
  * \f]
  * and the hessian of the Gauss-Newton approximation as
  * \f[
  *   \mathbf{H}_{gn}
  *   =
  *   \sum_i 2 \alpha_i \frac{\partial\underline{\bm{x}}_i}{\partial\bm{p}}^T \frac{\partial\underline{\bm{x}}_i}{\partial\bm{p}}
  *   +
  *   \sum_i 2 \beta_i \frac{\partial\underline{\bm{u}}_i}{\partial\bm{p}}^T \frac{\partial\underline{\bm{u}}_i}{\partial\bm{p}}
  * \f]
  * The reason why this is interesting is that the calculations can be performed
  * efficiently by iterating only over the values for which the coefficients
  * \f$ \alpha_i \f$ and \f$ \beta_i \f$ are non-zero.
  */
template<class Scalar>
class QuadraticObjective : public ObjectiveBase<Scalar> {
public:
/// \cond
  // Allows the use of VectorXs and MatrixXs
  PMPC_MATRIX_VECTOR_TYPEDEFS(Scalar);
/// \endcond

  /// Constructor, initializes all weights to zero.
  /** @param predictor a predictor instance that allows to perform predictions
    *   for a given model and control parameterization.
    */
  QuadraticObjective(const predictors::Recursive<Scalar>& predictor);

  /// Set all state weights to the same factor.
  /** This method set \f$ \mathbf{D}_{x,k} = w \mathbf{I} \f$.
    * @param w scalar weight to be used for all states. If negative, then
    *   \f$ \mathbf{D}_{x,k} \f$ is set to zero.
    */
  inline void setStateWeights(Scalar w) { setStateWeights(std::vector<Scalar>(alpha.size(), w)); }

  /// Changes the state weights to the specified ones.
  /** This method is equivalent to
    * setStateWeights(const Eigen::Ref<const VectorXs>),
    * but works with a std::vector instead. This can be useful in case you want
    * to use brace-initialization, like in setStateWeights({1,0}).
    * @param w weights of the states.
    * \see setStateWeights(const Eigen::Ref<const VectorXs>)
    */
  inline void setStateWeights(const std::vector<Scalar>& w) { setStateWeights(Eigen::Map<const VectorXs>(w.data(), w.size())); }

  /// Changes the state weights to the specified ones.
  /** @param w weights of the states. It can be a vector of size dim_x*np, in
    *   which case it consists in all the weights of the predicted states.
    *   In alternative, it can be a vector of size dim_x, in which case the
    *   same weights are used for successive prediction samples, *i.e.*,
    *   \f$ \mathbf{D}_{k,x} = \mathrm{diag}(\bm{w})\,\forall k \f$.
    *   If w has any other size, a DimensionError exception is thrown.
    *   Finally, note that if any value in w is negative, it is replaced
    *   internally by a zero.
    */
  void setStateWeights(const Eigen::Ref<const VectorXs> w);


  /// Set all control weights to the same factor.
  /** This method set \f$ \mathbf{D}_{u,k} = w \mathbf{I} \f$.
    * @param w scalar weight to be used for all controls. If negative, then
    *   \f$ \mathbf{D}_{u,k} \f$ is set to zero.
    */
  inline void setControlWeights(Scalar w) { setControlWeights(std::vector<Scalar>(beta.size(), w)); }

  /// Changes the control weights to the specified ones.
  /** This method is equivalent to
    * setControlWeights(const Eigen::Ref<const VectorXs>),
    * but works with a std::vector instead. This can be useful in case you want
    * to use brace-initialization, like in setControlWeights({1,0}).
    * @param w weights of the controls.
    * \see setControlWeights(const Eigen::Ref<const VectorXs>)
    */
  inline void setControlWeights(const std::vector<Scalar>& w) { setControlWeights(Eigen::Map<const VectorXs>(w.data(), w.size())); }

  /// Changes the control input weights to the specified ones.
  /** @param w weights of the inputs. It can be a vector of size dim_u*np, in
    *   which case it consists in all the weights of the control sequence.
    *   In alternative, it can be a vector of size dim_u, in which case the
    *   same weights are used for successive input samples, *i.e.*,
    *   \f$ \mathbf{D}_{k,u} = \mathrm{diag}(\bm{w})\,\forall k \f$.
    *   If w has any other size, a DimensionError exception is thrown.
    *   Finally, note that if any value in w is negative, it is replaced
    *   internally by a zero.
    */
  void setControlWeights(const Eigen::Ref<const VectorXs> w);

  /// Set the desired states \f$ \bm{x}_k^\star \f$.
  /** This method is equivalent to
    * setDesiredStates(const Eigen::Ref<const VectorXs>),
    * but works with a std::vector instead. This can be useful in case you want
    * to use brace-initialization, like in setDesiredStates({1,0}).
    * @param xdes desired state(s).
    * \see setDesiredStates(const Eigen::Ref<const VectorXs>)
    */
  inline void setDesiredStates(const std::vector<Scalar>& xdes) { setDesiredStates(Eigen::Map<const VectorXs>(xdes.data(), xdes.size())); }

  /// Set the desired states \f$ \bm{x}_k^\star \f$.
  /** @param xdes desired state(s). It can be a vector of size dim_x*np, in
    *   which case it consists in all the desired values of the predicted
    *   states. In alternative, it can be a vector of size dim_x, in which case
    *   the same values are used for successive prediction samples, *i.e.*,
    *   \f$ \bm{x}_k^\star = \bm{x}_{des}\,\forall k \f$.
    *   If xdes has any other size, a DimensionError exception is thrown.
    */
  inline void setDesiredStates(const Eigen::Ref<const VectorXs> xdes) { setSequence(xdes, x_desired, predictor.model.dim_x, "desired states"); }

  /// Set the desired control inputs \f$ \bm{u}_k^\star \f$.
  /** This method is equivalent to
    * setDesiredControls(const Eigen::Ref<const VectorXs>),
    * but works with a std::vector instead. This can be useful in case you want
    * to use brace-initialization, like in setDesiredControls({1,0}).
    * @param udes desired control(s).
    * \see setDesiredControls(const Eigen::Ref<const VectorXs>)
    */
  inline void setDesiredControls(const std::vector<Scalar>& udes) { setDesiredControls(Eigen::Map<const VectorXs>(udes.data(), udes.size())); }

  /// Set the desired control inputs \f$ \bm{u}_k^\star \f$.
  /** @param udes desired control(s). It can be a vector of size dim_u*np, in
    *   which case it consists in all the desired values of the control
    *   sequence. In alternative, it can be a vector of size dim_u, in which
    *   case the same values are used for successive control samples, *i.e.*,
    *   \f$ \bm{u}_k^\star = \bm{u}_{des}\,\forall k \f$.
    *   If udes has any other size, a DimensionError exception is thrown.
    */
  inline void setDesiredControls(const Eigen::Ref<const VectorXs> udes) { setSequence(udes, u_desired, predictor.model.dim_u, "desired controls"); }

  /// Returns the state weights.
  inline const VectorXs& stateWeights() const { return alpha; }

  /// Returns the control input weights.
  inline const VectorXs& controlWeights() const { return beta; }

  /// Returns the desired states \f$\underline{\bm{x}}^\star\f$.
  inline const VectorXs& desiredStates() const { return x_desired; }

  /// Returns the desired control inputs \f$\underline{\bm{u}}^\star\f$.
  inline const VectorXs& desiredControls() const { return u_desired; }

  /// Returns a Gauss-Newton approximation of this objective.
  void gaussNewtonApproximation(
    const Eigen::Ref<const VectorXs> x0,
    const Eigen::Ref<const VectorXs> p,
    Scalar& obj,
    Eigen::Ref<MatrixXs> Jobj,
    Eigen::Ref<MatrixXs> Hgn
  ) const;

  using ObjectiveBase<Scalar>::predictor;

protected:
  /// Specializes ObjectiveBase::objective_impl().
  /** This method provides an implementation that evaluates the objective as
    * \f[
    *   \sum_{k=1}^{n_p} (\bm{x}_k-\bm{x}_k^\star)^T \mathbf{D}_{x,k} (\bm{x}_k-\bm{x}_k^\star)
    *   + \sum_{k=0}^{n_p-1} (\bm{u}_k-\bm{u}_k^\star)^T \mathbf{D}_{u,k} (\bm{u}_k-\bm{u}_k^\star)
    * \f]
    * @param[in] x0 initial configuration of the system.
    * @param[in] p free-parameters of the control.
    * @param[out] obj value of the objective function at the current point.
    * @param[out] Jobj jacobian of the objective function with respect to
    *   the free-parameters. If nullptr is given, then the jacobian should
    *   not be computed in the function call.
    */
  virtual void evaluate_impl(
    const Eigen::Ref<const VectorXs> x0,
    const Eigen::Ref<const VectorXs> p,
    Scalar& obj,
    Eigen::Ref<MatrixXs>* Jobj
  ) const override;

  /// Vector of all state coefficients.
  /** There is one entry per predicted value, that is, the length of alpha
    * corresponds to dim_x * np.
    */
  VectorXs alpha;
  /// Indices of the states that are not to be ignored in the objective.
  std::vector<int> non_zero_states;
  /// Desired states for the optimization.
  /** There is one entry per predicted value, that is, the length of x_desired
    * corresponds to dim_x * np.
    */
  VectorXs x_desired;

  /// Vector of all control coefficients.
  /** There is one entry per control value, that is, the length of beta
    * corresponds to dim_u * np.
    */
  VectorXs beta;
  /// Indices of the controls that are not to be ignored in the objective.
  std::vector<int> non_zero_controls;
  /** There is one entry per control value, that is, the length of u_desired
    * corresponds to dim_u * np.
    */
  VectorXs u_desired;

private:
  /// Auxiliary method to set weights and desired values.
  /** The job of setStateWeights, setControlWeights, setDesiredStates and
    * setDesiredControls is rather similar: they check the dimension of the
    * input and, if it is appropriate, they set a target vector. This method
    * should therefore take care of this task on the abstract level, *i.e.*,
    * without actually knowing *what* it is filling.
    * @param source a vector of length sample_dim or np*sample_dim. If the input
    *   has size sample_dim, it is repeated np times and the function is called
    *   again.
    * @param dest where to copy the final vector, *e.g.*, alpha or beta.
    * @param sample_dim dimension of a "single sample". In fact, source is
    *   either a single segment of length sample_dim or np blocks of
    *   this length.
    * @param what name of what is being filled. This is just to throw a more
    *   informative exception if the sime of source is not compatible.
    */
  void setSequence(
    const Eigen::Ref<const VectorXs> source,
    VectorXs& dest,
    int sample_dim,
    const std::string& what
  );

  // Used for internal calculations. They are here to avoid allocating the
  // memory every time.
  mutable VectorXs xx, uu;
  mutable MatrixXs Jxx, Juu;
};

} // namespace optimization
} // namespace pmpc

