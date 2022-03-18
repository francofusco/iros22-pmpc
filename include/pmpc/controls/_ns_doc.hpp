namespace pmpc {
/// Definitions of control input parameterizations.
/** For the sake of MPC, an optimization algorithm has to find an optimal
  * control sequence that can steer the configuration towards a desired
  * reference. Given a prediction horizon \f$ n_p \f$, classical schemes
  * generally consider a control sequence in the form
  * \f$\underline{\bm{u}} = \bm{u}_0, \bm{u}_1, \cdots, \bm{u}_{n_p-1} \f$.
  * To find a trade-off between a long prediction horizon and small
  * optimization time, it is common to reduce the number of free control
  * samples to \f$ n_c \leq n_p \f$, with samples from \f$ n_c \f$ up to
  * \f$ n_p-1 \f$ generally kept equal to \f$ \bm{u}_{n_c-1} \f$.
  *
  * To extend this simple (yet powerful) idea, \f$ \pi MPC \f$ considers
  * a wider range of parameterizations, which provide the control sequence
  * \f$\underline{\bm{u}}\f$ as a function of some parameters \f$\bm{p}\f$.
  * Such function might be linear or nonlinear in the parameter vector. The
  * goal is to find some good lower-dimensional parameterizations that allow
  * to improve the tracking performances while keeping a reduced complexity.
  *
  * This namespace provides different classes that define parameterizations,
  * all deriving from the base Parameterization.
  */
namespace controls { }
}
