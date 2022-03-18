#pragma once

#ifndef PMPC_WITH_QPOASES
#error "qpOASES has not been detected. As such, this solver is not available."
#else

#include <pmpc/macros.hpp>
#include <pmpc/controls/linear.hpp>
#include <pmpc/predictors/recursive.hpp>
#include <pmpc/optimization/quadratic_objective.hpp>
#include <pmpc/optimization/linear_control_constraints.hpp>
#include <pmpc/optimization/nonlinear_solver_base.hpp>
#include <qpOASES.hpp>
#include <memory>

namespace pmpc {
namespace optimization {

/// Nonlinear solver based on qpOASES.
/** Solver
  * \todo Complete the detailed description.
  */
class SimplifiedGaussNewtonSolver : public NonlinearSolverBase<REFER_NAMESPACE_QPOASES real_t> {
public:
/// \cond
  // Make sure we can refer to real_t in a shorter way
  typedef REFER_NAMESPACE_QPOASES real_t Scalar;
  // Allows the use of VectorXs and MatrixXs
  PMPC_MATRIX_VECTOR_TYPEDEFS(Scalar);
/// \endcond

  /// Constructor.
  SimplifiedGaussNewtonSolver(
    const predictors::Recursive<Scalar>& predictor,
    const QuadraticObjective<Scalar>& objective
  );

protected:
  /// Solve the problem using a Sequential Quadratic Programming approach with Gauss-Newton approximation.
  virtual bool solve_impl(
    const Eigen::Ref<const VectorXs> x0,
    Eigen::Ref<VectorXs> p
  ) override;

  /// Check if constraints are linear.
  virtual void setupConstraints() override;

/// \cond
  // Dynamic-size, row-major matrix with entries of qpOASES's real_t type.
  typedef Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> RowMatrixXs;
/// \endcond

  std::unique_ptr<REFER_NAMESPACE_QPOASES SQProblem> solver; ///< Sequential QP solver from qpOASES.
  REFER_NAMESPACE_QPOASES Options sqp_options; ///< qpOASES' class to configure the solver.
  unsigned int num_vars; ///< Number of decision variables this problem was created for.
  unsigned int num_constr; ///< Number of constraints this problem was created for.
  MatrixXs H; ///< Hessian matrix of the problem.
  MatrixXs f; ///< Gradient of the problem (it is a 1-by-n matrix).
  RowMatrixXs C; ///< Constraint matrix of the problem.
  VectorXs d; ///< Constraint vector of the problem.
};

} // namespace optimization
} // namespace pmpc


#endif
