#pragma once

#include <pmpc/models/model_function.hpp>

namespace pmpc {
namespace models {
namespace discrete {

/// Abstract base class for all discrete time models.
/** This class represents a system in the form
  * \f[
  *   \bm{x}_{k+1} = \bm{f}(\bm{x}_k, \bm{u}_k)
  * \f]
  * where \f$ \bm{x}_k \f$ and \f$ \bm{x}_{k+1} \f$ are states of the system
  * at the current time sample \f$ k \f$ and at the next one.
  * \f$ \bm{u}_k \f$ is the control input that steers the system from
  * \f$\bm{x}_k\f$ to \f$\bm{x}_{k+1}\f$.
  *
  * The jacobians of the model write as:
  * \f[
  *   \mathbf{F}_x = \frac{\partial\bm{x}_{k+1}}{\partial\bm{x}_k}
  *   \qquad
  *   \mathbf{F}_u = \frac{\partial\bm{x}_{k+1}}{\partial\bm{u}_k}
  * \f]
  *
  * It specializes ModelFunction simply to provide a semantical distinction
  * from a continuous time model.
  */
template<class Scalar>
class Model : public ModelFunction<Scalar> {
public:
  /// Store information about dimensions.
  /** Simply calls ModelFunction(int,int).
    * \param dim_x dimension of the state vector of the system.
    * \param dim_u dimension of the control vector of the system.
    */
  Model(int dim_x, int dim_u) : ModelFunction<Scalar>(dim_x, dim_u) {}

};

} // discrete
} // models
} // pmpc
