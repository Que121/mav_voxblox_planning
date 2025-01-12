#include <math.h>
#include <stdio.h>
#include <algorithm>
#include <ostream>
#include <sstream>
#include <string>

#include "mav_trajectory_generation/timing_mgv.h"

namespace mgv_trajectory_generation
{
  namespace timing
  {

    Timing &Timing::Instance()
    {
      static Timing t;
      return t;
    }

    Timing::Timing() : max_tag_length_(0) {}

    Timing::~Timing() {}

    // Static functions to query the timers:
    size_t Timing::GetHandle(std::string const &tag)
    {
      // Search for an existing tag.
      map_t::iterator i = Instance().tag_map_.find(tag);
      if (i == Instance().tag_map_.end())
      {
        // If it is not there, create a tag.
        size_t handle = Instance().timers_.size();
        Instance().tag_map_[tag] = handle;
        Instance().timers_.push_back(TimerMapValue());
        // Track the maximum tag length to help printing a table of timing values
        // later.
        Instance().max_tag_length_ =
            std::max(Instance().max_tag_length_, tag.size());
        return handle;
      }
      else
      {
        return i->second;
      }
    }

    std::string Timing::GetTag(size_t handle)
    {
      std::string tag;

      // Perform a linear search for the tag.
      for (typename map_t::value_type current_tag : Instance().tag_map_)
      {
        if (current_tag.second == handle)
        {
          return current_tag.first;
        }
      }
      return tag;
    }

    // Class functions used for timing.
    Timer::Timer(size_t handle, bool constructStopped)
        : timing_(false), handle_(handle)
    {
      if (!constructStopped)
        Start();
    }

    Timer::Timer(std::string const &tag, bool constructStopped)
        : timing_(false), handle_(Timing::GetHandle(tag))
    {
      if (!constructStopped)
        Start();
    }

    Timer::~Timer()
    {
      if (IsTiming())
        Stop();
    }

    void Timer::Start()
    {
      timing_ = true;
      time_ = std::chrono::system_clock::now();
    }

    void Timer::Stop()
    {
      std::chrono::time_point<std::chrono::system_clock> now =
          std::chrono::system_clock::now();
      double dt = std::chrono::duration<double>(now - time_).count();

      Timing::Instance().AddTime(handle_, dt);
      timing_ = false;
    }

    bool Timer::IsTiming() const { return timing_; }

    void Timing::AddTime(size_t handle, double seconds)
    {
      timers_[handle].acc_.Add(seconds);
    }

    double Timing::GetTotalSeconds(size_t handle)
    {
      return Instance().timers_[handle].acc_.Sum();
    }
    double Timing::GetTotalSeconds(std::string const &tag)
    {
      return GetTotalSeconds(GetHandle(tag));
    }
    double Timing::GetMeanSeconds(size_t handle)
    {
      return Instance().timers_[handle].acc_.Mean();
    }
    double Timing::GetMeanSeconds(std::string const &tag)
    {
      return GetMeanSeconds(GetHandle(tag));
    }
    size_t Timing::GetNumSamples(size_t handle)
    {
      return Instance().timers_[handle].acc_.TotalSamples();
    }
    size_t Timing::GetNumSamples(std::string const &tag)
    {
      return GetNumSamples(GetHandle(tag));
    }
    double Timing::GetVarianceSeconds(size_t handle)
    {
      return Instance().timers_[handle].acc_.LazyVariance();
    }
    double Timing::GetVarianceSeconds(std::string const &tag)
    {
      return GetVarianceSeconds(GetHandle(tag));
    }
    double Timing::GetMinSeconds(size_t handle)
    {
      return Instance().timers_[handle].acc_.Min();
    }
    double Timing::GetMinSeconds(std::string const &tag)
    {
      return GetMinSeconds(GetHandle(tag));
    }
    double Timing::GetMaxSeconds(size_t handle)
    {
      return Instance().timers_[handle].acc_.Max();
    }
    double Timing::GetMaxSeconds(std::string const &tag)
    {
      return GetMaxSeconds(GetHandle(tag));
    }

    double Timing::GetHz(size_t handle)
    {
      return 1.0 / Instance().timers_[handle].acc_.RollingMean();
    }

    double Timing::GetHz(std::string const &tag) { return GetHz(GetHandle(tag)); }

    std::string Timing::SecondsToTimeString(double seconds)
    {
      char buffer[256];
      snprintf(buffer, sizeof(buffer), "%09.6f", seconds);
      return buffer;
    }

    void Timing::Print(std::ostream &out)
    {
      map_t &tagMap = Instance().tag_map_;

      if (tagMap.empty())
      {
        return;
      }

      out << "SM Timing\n";
      out << "-----------\n";
      for (typename map_t::value_type t : tagMap)
      {
        size_t i = t.second;
        out.width((std::streamsize)Instance().max_tag_length_);
        out.setf(std::ios::left, std::ios::adjustfield);
        out << t.first << "\t";
        out.width(7);

        out.setf(std::ios::right, std::ios::adjustfield);
        out << GetNumSamples(i) << "\t";
        if (GetNumSamples(i) > 0)
        {
          out << SecondsToTimeString(GetTotalSeconds(i)) << "\t";
          double meansec = GetMeanSeconds(i);
          double stddev = sqrt(GetVarianceSeconds(i));
          out << "(" << SecondsToTimeString(meansec) << " +- ";
          out << SecondsToTimeString(stddev) << ")\t";

          double minsec = GetMinSeconds(i);
          double maxsec = GetMaxSeconds(i);

          // The min or max are out of bounds.
          out << "[" << SecondsToTimeString(minsec) << ","
              << SecondsToTimeString(maxsec) << "]";
        }
        out << std::endl;
      }
    }
    std::string Timing::Print()
    {
      std::stringstream ss;
      Print(ss);
      return ss.str();
    }

    void Timing::Reset() { Instance().tag_map_.clear(); }

  } // namespace timing
} // namespace mav_trajectory_generation
