#pragma once

#include <pmpc/models/model_function.hpp>

namespace pmpc {
namespace models {
namespace continuous {

/// Abstract base class for all continuous time models.
/** This class represents a system in the form
  * \f[
  *   \dot{\bm{x}} = \bm{f}(\bm{x}, \bm{u})
  * \f]
  * where \f$ \bm{x} \f$ is the current state of the system, and
  * \f$ \dot{\bm{x}} \f$ its derivative. \f$ \bm{u} \f$ is instead the current
  * control input.
  *
  * The jacobians of the model write as:
  * \f[
  *   \mathbf{F}_x = \frac{\partial\dot{\bm{x}}}{\partial\bm{x}}
  *   \qquad
  *   \mathbf{F}_u = \frac{\partial\dot{\bm{x}}}{\partial\bm{u}}
  * \f]
  *
  * It specializes ModelFunction simply to provide a semantical distinction
  * from a discrete time model.
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

} // continuous
} // models
} // pmpc
