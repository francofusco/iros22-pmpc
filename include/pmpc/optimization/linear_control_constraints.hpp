#pragma once

#include <vector>
#include <pmpc/macros.hpp>
#include <pmpc/controls/linear.hpp>
#include <pmpc/optimization/control_constraints.hpp>


namespace pmpc {
namespace optimization {

/// Class that represents linear constraints on the control inputs.
template<class Scalar>
class LinearControlConstraints : public ControlConstraints<Scalar> {
public:
/// \cond
  // Allows the use of VectorXs and MatrixXs
  PMPC_MATRIX_VECTOR_TYPEDEFS(Scalar);
/// \endcond

  LinearControlConstraints(
    const controls::Linear<Scalar>& param
  );

  /// Remove all constraints.
  inline void resetConstraints() {
    constraints_changed = true;
    Cb = MatrixXs::Zero(0, param.dim_u);
    db = VectorXs::Zero(0);
    Cv = MatrixXs::Zero(0, param.dim_u);
    dv = VectorXs::Zero(0);
    C = MatrixXs::Zero(0, param.dim_p);
    d = VectorXs::Zero(0);
  }

  /// Add lower and upper bounds to the control.
  void addBounds(
    const Eigen::Ref<const VectorXs> lb,
    const Eigen::Ref<const VectorXs> ub
  );

  /// Add lower bounds to the control.
  void addLBounds(
    const Eigen::Ref<const VectorXs> lb
  );

  /// Add upper bounds to the control.
  void addUBounds(
    const Eigen::Ref<const VectorXs> ub
  );

  /// Add lower and upper bounds to the control.
  inline void addBounds(
    const std::vector<Scalar>& lb,
    const std::vector<Scalar>& ub
  )
  {
    addBounds(
      Eigen::Map<const VectorXs>(lb.data(), lb.size()),
      Eigen::Map<const VectorXs>(ub.data(), ub.size())
    );
  }

  /// Add lower bounds to the control.
  inline void addLBounds(
    const std::vector<Scalar>& lb
  )
  {
    addLBounds(Eigen::Map<const VectorXs>(lb.data(), lb.size()));
  }

  /// Add upper bounds to the control.
  inline void addUBounds(
    const std::vector<Scalar>& ub
  )
  {
    addLBounds(Eigen::Map<const VectorXs>(ub.data(), ub.size()));
  }

  /// Add lower and upper bounds to a specific control coordinate.
  inline void addBounds(
    int i,
    Scalar lb,
    Scalar ub
  );

  /// Add lower bounds to a specific control coordinate.
  void addLBounds(
    int i,
    Scalar lb
  );

  /// Add upper bounds to a specific control coordinate.
  void addUBounds(
    int i,
    Scalar ub
  );

  /// Add generic constraints in the form \f$\mathbf{C}\bm{u}_k\leq\mathbf{d}\f$.
  void addConstraints(
    const Eigen::Ref<const MatrixXs> C,
    const Eigen::Ref<const VectorXs> d
  );

  /// Add control variation bounds in the form \f$\left|\bm{u}_k - \bm{u}_{k-1}\right|\leq \Delta\mathbf{u}_{max}\f$.
  void addVariationBounds(
    const Eigen::Ref<const VectorXs> du_max
  );

  /// Add control variation bounds in the form \f$\left|\bm{u}_k - \bm{u}_{k-1}\right|\leq \Delta\mathbf{u}_{max}\f$.
  inline void addVariationBounds(
    const std::vector<Scalar>& du_max
  )
  {
    addVariationBounds(Eigen::Map<const VectorXs>(du_max.data(), du_max.size()));
  }

  /// Add control variation bounds to a specific control coordinate.
  void addVariationBounds(
    int i,
    Scalar ub
  );

  /// Internally update all matrices related to the constraints.
  virtual void update(
    const Eigen::Ref<const VectorXs> p
  );

  inline const MatrixXs& getC() const { return C; }
  inline const VectorXs& getD() const { return d; }

  inline const bool& constraintsChanged() const { return constraints_changed; }

  virtual int dim() const override { return C.rows(); }

  const controls::Linear<Scalar>& param;

protected:
  MatrixXs Cb; ///< Matrix of bound constraints of each control sample.
  VectorXs db; ///< Vector of bound constraints of each control sample.
  MatrixXs Cv; ///< Matrix of variation constraints of each control sample.
  VectorXs dv; ///< Vector of variation constraints of each control sample.
  MatrixXs C; ///< Matrix of all inequality constraints (for all samples).
  VectorXs d; ///< Vector of all inequality constraints (for all samples).

  bool constraints_changed; /// If true, constraints have been changed.

  virtual void evaluate_impl(
    const Eigen::Ref<const VectorXs> p,
    Eigen::Ref<VectorXs> g,
    Eigen::Ref<MatrixXs>* Jg
  ) const override;

};

} // namespace optimization
} // namespace pmpc

