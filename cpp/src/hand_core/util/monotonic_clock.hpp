#pragma once

#include <time.h>

namespace aidin_hand2
{

// Absolute nanoseconds, monotonic for intervals and realtime for timestamps

inline long now_nanoseconds()
{
  timespec now{};
  clock_gettime(CLOCK_MONOTONIC, &now);
  return now.tv_sec * 1000000000L + now.tv_nsec;
}

inline void sleep_until_nanoseconds(long absolute_nanoseconds)
{
  timespec target{};
  target.tv_sec  = absolute_nanoseconds / 1000000000L;
  target.tv_nsec = absolute_nanoseconds % 1000000000L;
  clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &target, nullptr);
}

// Epoch based, so it lines up with ROS header.stamp, rosbag and other sensors
inline long system_nanoseconds()
{
  timespec now{};
  clock_gettime(CLOCK_REALTIME, &now);
  return now.tv_sec * 1000000000L + now.tv_nsec;
}

}  // namespace aidin_hand2
