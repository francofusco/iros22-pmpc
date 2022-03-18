#pragma once

#ifndef PMPC_WITH_QPOASES
#error "qpOASES has not been found. As such, this solver is not available."
#else

#include <pmpc/macros.hpp>
#include <pmpc/predictors/linear.hpp>
#include <pmpc/optimization/quadratic_objective.hpp>
#include <pmpc/optimization/linear_solver_base.hpp>
#include <qpOASES.hpp>
#include <memory>

namespace pmpc {
namespace optimization {

/// QP solver for linear MPC problems.
/** \todo Write the detailed description of LinearSolverQPOASES.
  * \warning This class works only under the assumption that the controlled
  *   system is invariant. In particular, it assumes that the Hessian matrix
  *   of the QP problem and the constraints matrix do not change between
  *   subsequent iterations. The initial state can change, since it affects
  *   only the gradient of the objective and the constraints vector.
  */
class LinearSolverQPOASES : public LinearSolverBase<REFER_NAMESPACE_QPOASES real_t> {
public:
/// \cond
  // Allows the use of VectorXs and MatrixXs
  PMPC_MATRIX_VECTOR_TYPEDEFS(REFER_NAMESPACE_QPOASES real_t);
/// \endcond

  /// Constructor, simply calls the constructor of its parent class.
  LinearSolverQPOASES(
    const predictors::Linear<REFER_NAMESPACE_QPOASES real_t>& predictor,
    const QuadraticObjective<REFER_NAMESPACE_QPOASES real_t>& objective
  );

protected:
  /// Solve the QP problem using qpOASES.
  virtual bool solve_impl(
    const Eigen::Ref<const VectorXs> x0,
    Eigen::Ref<VectorXs> p
  ) override;

/// \cond
  // Dynamic-size, row-major matrix with entries of qpOASES's real_t type.
  typedef Eigen::Matrix<REFER_NAMESPACE_QPOASES real_t,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> RowMatrixXs;
/// \endcond

  std::unique_ptr<REFER_NAMESPACE_QPOASES QProblem> solver; ///< QP solver from qpOASES.
  REFER_NAMESPACE_QPOASES Options qp_options; ///< qpOASES' class to configure the solver.
  unsigned int num_vars; ///< Number of decision variables this problem was created for.
  unsigned int num_constr; ///< Number of constraints this problem was created for.
  RowMatrixXs H; ///< Hessian matrix of the problem.
  VectorXs f; ///< Gradient of the problem.
  RowMatrixXs Crm; ///< Constraint matrix of the problem (in row-major format).
};

} // namespace optimization
} // namespace pmpc


#endif
