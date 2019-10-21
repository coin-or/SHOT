// Copyright (C) 2009-2016 Benoit Chachuat, Imperial College London.
// All Rights Reserved.
// This code is published under the Eclipse Public License.

#ifndef MC__MCTIME_HPP
#define MC__MCTIME_HPP

#include <sys/resource.h>
#include <sys/times.h>

namespace mc
{

inline double
cpuclock()
{
  // Return current time [seconds]
  struct rusage ruse;
  getrusage (RUSAGE_SELF, &ruse);
  return ((double)(ruse.ru_utime.tv_sec + ruse.ru_utime.tv_usec / 1e6));

  //timeval time;
  //gettimeofday(&time, 0) ;
  //return time.tv_sec + time.tv_usec*1e-6; }
}

} // namespace mc
#endif
