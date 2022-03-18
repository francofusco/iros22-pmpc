#pragma once

#include <pmpc/macros.hpp>
#include <pmpc/models/discrete/model.hpp>

namespace pmpc {
namespace models {
namespace discrete {

/// Time invariant linear affine system.
/** This class represents linear time invariant systems whose evolution is
  * described by the set of difference equations
  * \f[
  *   \bm{x}_{k+1} = \mathbf{A}\bm{x}_k + \mathbf{B}\bm{u}_k + \bm{w}
  * \f]
  */
template<class Scalar>
class Linear : public Model<Scalar> {
public:
  // Allows the use of VectorXs and MatrixXs
  PMPC_MATRIX_VECTOR_TYPEDEFS(Scalar);

  using Model<Scalar>::dim_x;
  using Model<Scalar>::dim_u;

  /// "Dummy" constructor that only allocate the memory for the model matrices.
  /** This constructor creates a linear system whose matrices \f$\mathbf{A}\f$,
    * \f$\mathbf{b}\f$ and the affine vector \f$\bm{w}\f$ are all filled with
    * zeros. Their dimensions are `dim_x`-by-`dim_x`, `dim_x`-by-`dim_u` and
    * `dim_x` respectively.
    * @param dim_x state dimension.
    * @param dim_u control input dimension.
    */
  Linear(int dim_x, int dim_u);

  /// Creates a linear system with no affine term.
  /** This constructor creates a linear system from given matrices
    * \f$\mathbf{A}\f$ and \f$\mathbf{B}\f$. The affine vector \f$\bm{w}\f$ is
    * filled with zeros.
    * @param A state matrix. Must be squared.
    * @param B control matrix. Must have as many rows as `A`.
    */
  Linear(
    const Eigen::Ref<const MatrixXs>& A,
    const Eigen::Ref<const MatrixXs>& B
  );

  /// Creates a linear affine system.
  /** This constructor creates a linear system from given matrices
    * \f$\mathbf{A}\f$, \f$\mathbf{B}\f$ and affine vector \f$\bm{w}\f$.
    * @param A state matrix. Must be squared.
    * @param B control matrix. Must have as many rows as `A`.
    * @param W affine vector. Must have as many rows as `A`.
    */
  Linear(
    const Eigen::Ref<const MatrixXs>& A,
    const Eigen::Ref<const MatrixXs>& B,
    const Eigen::Ref<const VectorXs>& W
  );

  /// Returns the state matrix \f$\mathbf{A}\f$.
  inline const MatrixXs& getA() const { return A; }
  /// Returns the control matrix \f$\mathbf{B}\f$.
  inline const MatrixXs& getB() const { return B; }
  /// Returns the affine vector \f$\bm{w}\f$.
  inline const VectorXs& getW() const { return W; }

protected:
  /// Specialization of Model<Scalar>::function_impl().
  virtual void function_impl(
    const Eigen::Ref<const VectorXs> x,
    const Eigen::Ref<const VectorXs> u,
    Eigen::Ref<VectorXs> f,
    Eigen::Ref<MatrixXs>* Fx,
    Eigen::Ref<MatrixXs>* Fu
  ) const override;

  MatrixXs A; ///< State matrix \f$\mathbf{A}\f$.
  MatrixXs B; ///< Control matrix \f$\mathbf{B}\f$.
  VectorXs W; ///< Affine vector \f$\bm{w}\f$.
};

} // discrete
} // models
} // pmpc

