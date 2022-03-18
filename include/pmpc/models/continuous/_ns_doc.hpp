namespace pmpc {
namespace models {

/// Definition of continuous time models.
/** This namespace groups all classes that can be used to describe
  * dynamical systems in continuous time.
  *
  * The fundamental class is continuous::Model, which inherits from
  * ModelFunction. It is used to represent a system in the form
  * \f[
  *   \dot{\bm{x}} = \bm{f}(\bm{x}, \bm{u})
  * \f]
  * where \f$ \bm{x} \f$ is the current state of the system, and
  * \f$ \dot{\bm{x}} \f$ its derivative. \f$ \bm{u} \f$ is instead the current
  * control input.
  *
  * The jacobians of the model are finally:
  * \f[
  *   \mathbf{F}_x = \frac{\partial\dot{\bm{x}}}{\partial\bm{x}}
  *   \qquad
  *   \mathbf{F}_u = \frac{\partial\dot{\bm{x}}}{\partial\bm{u}}
  * \f]
  */
namespace continuous { }

}
}
