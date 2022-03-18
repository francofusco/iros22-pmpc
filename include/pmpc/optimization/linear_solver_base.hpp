#pragma once

#include <pmpc/macros.hpp>
#include <pmpc/predictors/linear.hpp>
#include <pmpc/optimization/quadratic_objective.hpp>
#include <pmpc/optimization/linear_control_constraints.hpp>

// TEMP: to be kept until all methods are properly implemented.
#include <pmpc/devel.hpp>

namespace pmpc {
namespace optimization {

template<class Scalar>
class LinearSolverBase {
public:
/// \cond
  // Allows the use of VectorXs and MatrixXs
  PMPC_MATRIX_VECTOR_TYPEDEFS(Scalar);
  // Define the diagonal matrix type (used for Dx and Du).
  typedef Eigen::DiagonalMatrix<Scalar,Eigen::Dynamic> DMatrixXs;
/// \endcond

  LinearSolverBase(
    const predictors::Linear<Scalar>& predictor,
    const QuadraticObjective<Scalar>& objective
  );

  /// Remove all state inequality constraints.
  inline void resetStateConstraints() WARN_NOT_IMPLEMENTED;

  /// Remove all control inequality constraints.
  inline void resetControlConstraints() {
    if(control_constraints) {
      control_constraints->resetConstraints();
    }
  }

  /// Remove all constraints.
  inline void resetConstraints() {
    resetStateConstraints();
    resetControlConstraints();
  }

  /// Add lower and upper bounds to the states.
  void addStateBounds(
    const Eigen::Ref<const VectorXs> lb,
    const Eigen::Ref<const VectorXs> ub
  ) NOT_IMPLEMENTED;

  /// Add lower bounds to the states.
  void addStateLBounds(
    const Eigen::Ref<const VectorXs> lb
  ) NOT_IMPLEMENTED;

  /// Add upper bounds to the states.
  void addStateUBounds(
    const Eigen::Ref<const VectorXs> ub
  ) NOT_IMPLEMENTED;

  /// Add lower and upper bounds to a specific state coordinate.
  void addStateBounds(
    int i,
    Scalar lb,
    Scalar ub
  ) NOT_IMPLEMENTED;

  /// Add lower bounds to a specific state coordinate.
  void addStateLBounds(
    int i,
    Scalar lb
  ) NOT_IMPLEMENTED;

  /// Add upper bounds to a specific state coordinate.
  void addStateUBounds(
    int i,
    Scalar ub
  ) NOT_IMPLEMENTED;


  /// Set the instance used to handle control input constraints.
  void setControlConstraintsInstance(
    LinearControlConstraints<Scalar>& constraints
  );

  inline void removeControlConstraintsInstance() { control_constraints = nullptr; }

  /// Add lower and upper bounds to the control.
  inline void addControlBounds(
    const Eigen::Ref<const VectorXs> lb,
    const Eigen::Ref<const VectorXs> ub
  )
  {
    if(control_constraints)
      control_constraints->addBounds(lb, ub);
  }

  /// Add lower bounds to the control.
  inline void addControlLBounds(
    const Eigen::Ref<const VectorXs> lb
  )
  {
    if(control_constraints)
      control_constraints->addLBounds(lb);
  }

  /// Add upper bounds to the control.
  inline void addControlUBounds(
    const Eigen::Ref<const VectorXs> ub
  )
  {
    if(control_constraints)
      control_constraints->addUBounds(ub);
  }

  /// Add lower and upper bounds to the control.
  inline void addControlBounds(
    const std::vector<Scalar>& lb,
    const std::vector<Scalar>& ub
  )
  {
    if(control_constraints)
      control_constraints->addBounds(lb, ub);
  }

  /// Add lower bounds to the control.
  inline void addControlLBounds(
    const std::vector<Scalar>& lb
  )
  {
    if(control_constraints)
      control_constraints->addLBounds(lb);
  }

  /// Add upper bounds to the control.
  inline void addControlUBounds(
    const std::vector<Scalar>& ub
  )
  {
    if(control_constraints)
      control_constraints->addUBounds(ub);
  }

  /// Add lower and upper bounds to a specific control coordinate.
  inline void addControlBounds(
    int i,
    Scalar lb,
    Scalar ub
  )
  {
    if(control_constraints)
      control_constraints->addBounds(i, lb, ub);
  }

  /// Add lower bounds to a specific control coordinate.
  inline void addControlLBounds(
    int i,
    Scalar lb
  )
  {
    if(control_constraints)
      control_constraints->addLBounds(i, lb);
  }

  /// Add upper bounds to a specific control coordinate.
  inline void addControlUBounds(
    int i,
    Scalar ub
  )
  {
    if(control_constraints)
      control_constraints->addUBounds(i, ub);
  }

  /// Add generic constraints in the form \f$\mathbf{C}_u\bm{u}_k\leq\mathbf{d}_u\f$.
  inline void addControlConstraints(
    const Eigen::Ref<const MatrixXs> Cu,
    const Eigen::Ref<const VectorXs> du
  )
  {
    if(control_constraints)
      control_constraints->addConstraints(Cu, du);
  }

  bool solve(
    const Eigen::Ref<const VectorXs> x0,
    Eigen::Ref<VectorXs> p,
    bool update_models = true
  );

  const predictors::Linear<Scalar>& predictor;
  const QuadraticObjective<Scalar>& objective;

protected:
  virtual bool solve_impl(
    const Eigen::Ref<const VectorXs> x0,
    Eigen::Ref<VectorXs> p
  ) = 0;

  DMatrixXs Dx; ///< Matrix that contains the weights of the objective (state).
  DMatrixXs Du; ///< Matrix that contains the weights of the objective (control).
  MatrixXs Q; ///< Matrix of the quadratic objective.
  VectorXs r; ///< Vector of the quadratic objective.
  VectorXs r_red; ///< Vector of the quadratic objective, minus \f$\tilde{\mathbf{A}}\bm{x}_0\f$.

  MatrixXs C; ///< Matrix of all inequality constraints.
  VectorXs d; ///< Vector of all inequality constraints.

  optimization::LinearControlConstraints<Scalar>* control_constraints;

private:
  /// Updates the matrices and vectors that represent the objective and/or the constraints.
  void update(
    const Eigen::Ref<const VectorXs> x0,
    const Eigen::Ref<const VectorXs> p,
    bool full_update
  );
};

} // namespace optimization
} // namespace pmpc

