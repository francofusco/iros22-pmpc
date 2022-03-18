namespace pmpc {
namespace models {

/// Definition of discrete time models.
/** This namespace groups all classes that can be used to describe
  * dynamical systems in discrete time.
  *
  * The fundamental class is discrete::Model, which inherits from
  * ModelFunction. It is used to represent a system in the form
  * \f[
  *   \bm{x}_{k+1} = \bm{f}(\bm{x}_k, \bm{u}_k)
  * \f]
  * where \f$ \bm{x}_k \f$ is the current state of the system, and
  * \f$ \bm{u}_k \f$ is instead the current control input.
  * \f$ \bm{x}_{k+1} \f$ represents the state reached at the next discrete time
  * step.
  *
  * The jacobians of the model correspond to:
  * \f[
  *   \mathbf{F}_x = \frac{\partial\bm{x}_{k+1}}{\partial\bm{x}_k}
  *   \qquad
  *   \mathbf{F}_u = \frac{\partial\bm{x}_{k+1}}{\partial\bm{u}_k}
  * \f]
  *
  *
  * ### Discretization of continuous-time models
  *
  * Almost always, a MPC scheme has to be designed to control a continuous-time
  * model. However, direct methods for MPC generally work on a discrete-time
  * model. It is thus common to use some discretization technique to generate
  * the discrete-time model to be used by the MPC optimizer from a continuous
  * one. In practice, you have the following options:
  *
  * - EulerIntegrator: the simplest technique, a forward Euler's integration
  *   method.
  * - RK4Integrator: an explicit Runge-Kutta integration method of order 4.
  * - LinearDiscretized: discretization of a linear affine system via matrix
  *   exponentiation.
  *
  * Note that all discretization techniques are provided in the form of a
  * discrete::Model subclass that "operate" on some continuous::Model instance.
  * As such, if you wish to provide any kind of custom discretization for a
  * system, you just need to write a specific sub-class of discrete::Model.
  */
namespace discrete { }

}
}
