#pragma once

#include <iostream>
#include <pmpc/macros.hpp>
#include <pmpc/exceptions.hpp>

/// Provide an implementation to a function/method so that it throws an exception.
#define NOT_IMPLEMENTED { throw NotImplemented(std::string(__FILE__) + " line " + std::to_string(__LINE__) + ": TO BE IMPLEMENTED"); }

/// Provide an implementation to a function/method so that it outputs a warning.
#define WARN_NOT_IMPLEMENTED { std::cout << "+--------------------------------" << std::endl << "|  !!! WARNING !!!" << std::endl << "|  " << std::string(__FILE__) << std::endl << "|  Line " << std::to_string(__LINE__) << std::endl << "|  CURRENTLY A NO-OP" << std::endl << "+--------------------------------" << std::endl; }
