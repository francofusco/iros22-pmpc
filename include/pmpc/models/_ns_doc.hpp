namespace pmpc {
/// Definition of continuous and discrete time models.
/** This namespace groups all classes that can be used to describe
  * dynamical systems, either in continuous or in discrete time.
  *
  * The fundamental class, which is the abstract base for almost all other
  * classes contained in this namespace, is ModelFunction. It is used to
  * represent a generic vector-valued function in the form
  * \f[
  *   \bm{f} = \bm{f}(\bm{x}, \bm{u})
  * \f]
  * where the meaning of the different quantities slightly changes depending on
  * the class of the model (continuous or discrete).
  *
  * In particular, two classes inherit from it: continuous::Model and
  * discrete::Model. The former is used to represent a set of differential
  * equations that describe the continuous time dynamics of the system, while
  * the latter corresponds to a set of difference equations describing the
  * evolution of discrete time systems. Even though the two kind of models
  * formally coincide, a semantic distinction has been made. This leads to
  * few duplicated classes which could have shared the source code. However,
  * we believe that the distinction allows to structure the code in a clearer
  * way.
  */
namespace models {}
}
