namespace pmpc {
/// Definitions of classes that can perform predictions for the state of a model.
/** Given a discrete-time model and a control parameterization, it is possible
  * to predict a number of future states that a system will visit over the
  * given prediction horizon. The classes provided in this namespace perform
  * this task by recursively evaluating the sequence of predictes states
  * \f$\bm{x}_1, \bm{x}_2, \cdots\f$ starting from a given initial state
  * \f$\bm{x}_0\f$.
  *
  * A special case is the one of linear models using linear parameterizations.
  * In that case, the whole prediction vector can be written as a linear-affine
  * combination of the parmeters and of the initial state.
  */
namespace predictors { }
}
