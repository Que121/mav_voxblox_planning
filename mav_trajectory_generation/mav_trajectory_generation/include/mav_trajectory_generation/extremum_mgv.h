#ifndef MGV_TRAJECTORY_GENERATION_EXTREMUM_H_
#define MGV_TRAJECTORY_GENERATION_EXTREMUM_H_

#include <iostream>

namespace mgv_trajectory_generation
{

  // Container holding the properties of an extremum (time, value,
  // segment where it occurred).
  struct Extremum
  {
  public:
    Extremum() : time(0.0), value(0.0), segment_idx(0) {}

    Extremum(double _time, double _value, int _segment_idx)
        : time(_time), value(_value), segment_idx(_segment_idx) {}

    bool operator<(const Extremum &rhs) const { return value < rhs.value; }
    bool operator>(const Extremum &rhs) const { return value > rhs.value; }

    double
        time;        // Time where the extremum occurs, relative to the segment start.
    double value;    // Value of the extremum at time.
    int segment_idx; // Index of the segment where the extremum occurs.
  };

  inline std::ostream &operator<<(std::ostream &stream, const Extremum &e)
  {
    stream << "time: " << e.time << ", value: " << e.value
           << ", segment idx: " << e.segment_idx << std::endl;
    return stream;
  }

} // namespace mgv_trajectory_generation

#endif // MGV_TRAJECTORY_GENERATION_EXTREMUM_H_
