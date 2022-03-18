#pragma once

#include <pmpc/macros.hpp>
#include <pmpc/predictors/recursive.hpp>
#include <pmpc/optimization/objective_base.hpp>
#include <pmpc/optimization/control_constraints.hpp>

// TEMP: to be kept until all methods are properly implemented.
#include <pmpc/devel.hpp>

namespace pmpc {
namespace optimization {

/// Base class for all nonlinear optimization algorithms.
/** \todo Long description.
  * \todo An "update" method that updates constraints before the optimization.
  */
template<class Scalar>
class NonlinearSolverBase {
public:
/// \cond
  // Allows the use of VectorXs and MatrixXs
  PMPC_MATRIX_VECTOR_TYPEDEFS(Scalar);
/// \endcond

  NonlinearSolverBase(
    const predictors::Recursive<Scalar>& predictor,
    const ObjectiveBase<Scalar>& objective
  );

  /// Set the instance used to handle control input constraints.
  void setControlConstraintsInstance(
    ControlConstraints<Scalar>& constraints
  );

  void removeControlConstraintsInstance();

  bool solve(
    const Eigen::Ref<const VectorXs> x0,
    Eigen::Ref<VectorXs> p,
    bool update_models = true
  );

  inline const std::string& lastError() const { return last_error; }

  const predictors::Recursive<Scalar>& predictor;
  const ObjectiveBase<Scalar>& objective;

protected:
  virtual bool solve_impl(
    const Eigen::Ref<const VectorXs> x0,
    Eigen::Ref<VectorXs> p
  ) = 0;

  /// To be specialized in sub-classes if they need to do something when constraints are added.
  virtual void setupConstraints() { }

  optimization::ControlConstraints<Scalar>* control_constraints;

  std::string last_error;

};

} // namespace optimization
} // namespace pmpc

