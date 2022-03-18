#pragma once
#include <vector>
#include <string>
#include <stdexcept>

/// Returns a vector of uniformely spaced values.
/** @param start initial value of the range.
  * @param stop final value of the range.
  * @param step step size between subsequent values in the range.
  * @return a vector of values in the form
  *   `[start, start+step, start+2*step, ...]`. Note that the last value is
  *   strictly smaller than stop.
  */
template<class T=int>
std::vector<T> range(T start, T stop, T step) {
  if(step <= 0)
    throw std::runtime_error("Step must be positive");
  std::vector<T> v;
  v.reserve((int)((stop-start)/step));
  for(T i=start; i<stop; i+=step) {
    v.push_back(i);
  }
  return v;
}


/// Returns a string representing a (printable) progressbar.
/** @param current an integer representing the progress.
  * @param max an integer representing maximum progress.
  * @param fill character used to populate the progressbar.
  * @return a string that:
  *   - Begins with a square bracket;
  *   - Contains the symbol `fill` repeated `current` times;
  *   - Contains `max-current` empty spaces;
  *   - Ends with a square bracket.
  *   As an example, `progressbar(2,5,'#')` would return the string `[##   ]`.
  * @note If `current` is greater than or equal to `max`, a completely filled
  *   bar of length `max` is returned.
  */
std::string progressbar(
  unsigned int current,
  unsigned int max,
  char fill
)
{
  std::string out("[");
  for(unsigned int i=0; i<current; i++)
    out += fill;
  for(unsigned int i=current; i<max; i++)
    out += " ";
  return out + "]";
}


char progressChar(unsigned int i)
{
  static char progress[] = {'|', '\\', '-', '/'};
  return progress[i%4];
}


/// Returns the average of a vector.
template<class T>
T average(const std::vector<T>& data) {
  if(data.size() == 0)
    return 0;
  T sum = 0;
  for(const auto& x : data)
    sum += x;
  return sum / data.size();
}
