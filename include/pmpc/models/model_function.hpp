#pragma once

#include <pmpc/macros.hpp>
#include <pmpc/exceptions.hpp>

namespace pmpc {
namespace models {

/// Abstract base class for all dynamical models.
/** \tparam Scalar datatype used to represent scalar values, *e.g*, `double`.
  *
  * This class represents a generic vector-valued function with two inputs:
  *
  * \f[ \bm{f} : \mathbb{R}^n \times \mathbb{R}^m \rightarrow \mathbb{R}^n \f]
  *
  * The function should be continuous and twice-differentiable.
  * The inputs are namely \f$ \bm{x} \f$ and \f$ \bm{u} \f$, representing
  * respectively a state and a control sample for a dynamical system. Depending
  * on the context, the value of \f$ \bm{f} \f$ assumes different meanings:
  * - For continuous time systems,
  *   \f$ \dot{\bm{x}} = \bm{f}(\bm{x}, \bm{u}) \f$, *i.e.*, it corresponds to
  *   the derivatives of the state vector.
  * - For discrete time systems,
  *   \f$ \bm{x}_{k+1} = \bm{f}(\bm{x}_k, \bm{u}_k) \f$, *i.e.*, it provides
  *   the state sample at the next discrete-time step.
  *
  * This class provides a common interface to all models. It allows the
  * evaluation of system's function as well as its jacobians, which are denoted
  * as:
  * \f[
  *   \mathbf{F}_x(\bm{x},\bm{u}) = \frac{\partial\bm{f}}{\partial\bm{x}}
  *   \qquad
  *   \mathbf{F}_u(\bm{x},\bm{u}) = \frac{\partial\bm{f}}{\partial\bm{u}}
  * \f]
  *
  * \todo Hessians are, currently, not implemented.
  *
  *
  * ### Implementation Requirements
  *
  * When implementing sub-classes, there are few things you might want to
  * consider to ensure a nice an correct workflow. These tips apply both in
  * the case of continuous and discrete systems, which is why they are reported
  * here.
  *
  * - Write `PMPC_MATRIX_VECTOR_TYPEDEFS(Scalar)` to have both VectorXs and
  *   MatrixXs typedef'd. They corresond to Eigen's dynamic-sized vectors and
  *   matrices templated on the type Scalar.
  * - To have access to members (both variables and methods) declared in parent
  *   classes, write `using Base::member`, where `Base` is the name of the
  *   parent class (including the template parameter)
  *   and `member` is the variable/method you want to pull into the local class
  *   scope. As an example, to access the number of coordinates in the state
  *   vector of your model you can type: using ModelFunction<Scalar>::dim_x.
  * - Provide an implementation of `function_impl()`. For this, you need to, at
  *   least, compute the value of the function inside it. Additionally, if the
  *   last function parameters are not `nullptr`, you can include the code that
  *   evaluates the jacobians \f$\mathbf{F}_x\f$ and \f$\mathbf{F}_u\f$. Note
  *   that writing the code of the jacobians is not mandatory, since they can
  *   be evaluated using Automatic Differentiation (see
  *   continuous::AutoDiffModel and discrete::AutoDiffModel). If you decide
  *   not to provide the jacobians, at least you should throw an exception of
  *   type pmpc::NotImplemented. This allows dependent classes to detect
  *   whether jacobians can be computed or not.
  */
template<class Scalar>
class ModelFunction {
public:
/// \cond
  // Allows the use of VectorXs and MatrixXs
  PMPC_MATRIX_VECTOR_TYPEDEFS(Scalar);
/// \endcond

  /// Store information about dimensions.
  /** \param dim_x dimension of the state vector of the system.
    * \param dim_u dimension of the control vector of the system.
    */
  ModelFunction(int dim_x, int dim_u);

  /// Virtual destructor, added as a best practice.
  virtual ~ModelFunction() = default;

  const int dim_x; ///< State dimension.
  const int dim_u; ///< Input dimension.

  /// Convenience method that returns a zero-valued state vector.
  inline VectorXs XZero() const { return VectorXs::Zero(dim_x); }
  /// Convenience method that returns a zero-valued control vector.
  inline VectorXs UZero() const { return VectorXs::Zero(dim_u); }
  /// Convenience method that returns a zero-valued state jacobian matrix.
  inline MatrixXs FXZero() const { return MatrixXs::Zero(dim_x,dim_x); }
  /// Convenience method that returns a zero-valued control jacobian matrix.
  inline MatrixXs FUZero() const { return MatrixXs::Zero(dim_x,dim_u); }

  /// System's function.
  /** This method allows to evaluate the model-specific function
    * \f$ \bm{f} \f$ at a given state and control.
    *
    * \param[in] x vector-like object of dimension dim_x.
    * \param[in] u vector-like object of dimension dim_x.
    * \param[out] f value of \f$ \bm{f}(\bm{x}, \bm{u}) \f$ (vector-like object
    *   of dimension dim_x).
    */
  inline void function(
    const Eigen::Ref<const VectorXs> x,
    const Eigen::Ref<const VectorXs> u,
    Eigen::Ref<VectorXs> f
  ) const
  {
    function_impl(x, u, f, nullptr, nullptr);
  }

  /// System's function and corresponding jacobians.
  /** This method allows to evaluate the model-specific function
    * \f$ \bm{f} \f$ at a given state and control, as well as its jacobians
    * with respect to a the state and control input.
    *
    * \param[in] x vector-like object of dimension dim_x.
    * \param[in] u vector-like object of dimension dim_x.
    * \param[out] f value of \f$ \bm{f}(\bm{x}, \bm{u}) \f$ (vector-like object
    *   of dimension dim_x).
    * \param[out] Fx matrix-like object of dimensions dim_x by dim_x,
    *   corresponding to \f$ \frac{\partial\bm{f}}{\partial\bm{x}} \f$
    *   evaluated at the given state and control.
    * \param[out] Fu matrix-like object of dimensions dim_x by dim_u,
    *   corresponding to \f$ \frac{\partial\bm{f}}{\partial\bm{u}} \f$
    *   evaluated at the given state and control.
    */
  inline void function(
    const Eigen::Ref<const VectorXs> x,
    const Eigen::Ref<const VectorXs> u,
    Eigen::Ref<VectorXs> f,
    Eigen::Ref<MatrixXs> Fx,
    Eigen::Ref<MatrixXs> Fu
  ) const
  {
    function_impl(x, u, f, &Fx, &Fu);
  }

protected:
  /// Low-level method used to evaluate system's function and jacobians.
  /** This method has to be implemented in sub-classes, allowing to evaluate
    * both the function \f$\bm{f}\f$ and its jacobians \f$\mathbf{F}_x\f$,
    * \f$\mathbf{F}_u\f$.
    *
    * \param[in] x current state, as a vector-like object of dimension dim_x.
    * \param[in] u current control, as a vector-like object of dimension dim_x.
    * \param[out] f value of the function \f$\bm{f}(\bm{x},\bm{u})\f$, as a
    *   vector-like object of dimension dim_x.
    * \param[out] Fx,Fu jacobians \f$\mathbf{F}_x\f$ and \f$\mathbf{F}_u\f$, as
    *   matrix-like objects of dimensions dim_x by dim_x and dim_x by dim_u
    *   respectively. Note that they are passed as pointers: if any of these is
    *   given as nullptr, then they are not to be computed - this corresponds
    *   to a case in which only the function evaluation is necessary.
    *   Note that you can decide not to provide the jacobians at all. In this
    *   case, the method should throw a pmpc::NotImplemented exception whenever
    *   a call to this method is performed with Fx and/or Fu not being nullptr.
    */
  virtual void function_impl(
    const Eigen::Ref<const VectorXs> x,
    const Eigen::Ref<const VectorXs> u,
    Eigen::Ref<VectorXs> f,
    Eigen::Ref<MatrixXs>* Fx,
    Eigen::Ref<MatrixXs>* Fu
  ) const = 0;
};


} // namespace models
} // namespace pmpc

