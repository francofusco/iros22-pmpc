#pragma once

#include <exception>

namespace pmpc {

/// Exception thrown when there is something wrong with a dimension inside a Model.
class DimensionError : public std::runtime_error {
public:
  DimensionError() = default;
  DimensionError(const char* msg) : std::runtime_error(msg) {}
  DimensionError(const std::string& msg) : std::runtime_error(msg) {}
};

/// Exception thrown when a function has not been implemented.
/** The primary goal of this class is to allow implementing sub-classes of
  * ModelFunction or similar without necessarily having to override
  * jacobians-related methods.
  * Code fragments that need the jacobians
  * can thus check if a particular class is able to evaluate them by checking
  * if the relevant method throws the exception or not.
  */
class NotImplemented : public std::runtime_error {
public:
  NotImplemented() = default;
  NotImplemented(const char* msg) : std::runtime_error(msg) {}
  NotImplemented(const std::string& msg) : std::runtime_error(msg) {}
};

/// Exception thrown when two pointers do not coincide.
/** Several classes in this library rely on dependent objects via the use of
  * references. To prevent some issues to arise, some components might check
  * if two references to, supposedly, the same object are actually different.
  * In this case, they will throw this exception to let the user better track
  * the error source.
  */
class DifferentInstances : public std::runtime_error {
public:
  DifferentInstances() = delete;
  DifferentInstances(const char* msg) : std::runtime_error(msg) {}
  DifferentInstances(const std::string& msg) : std::runtime_error(msg) {}
};

/// Exception thrown when failing a dynamic cast.
/** Some classes in this library rely on dependent objects of specific sub-type.
  * As an example, some solvers might work only if a given parameterization is
  * linear. As a consequence, some components might need to dynamically cast
  * objects of a given parent class to a specific derived class. If this is not
  * possible, then this exception is thrown.
  */
class BadType : public std::runtime_error {
public:
  BadType() = default;
  BadType(const char* msg) : std::runtime_error(msg) {}
  BadType(const std::string& msg) : std::runtime_error(msg) {}
};



} // namespace pmpc
