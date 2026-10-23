// Copyright (C) Benoit Chachuat, Imperial College London.
// All Rights Reserved.
// This code is published under the EPL-2.0 with GPL-2.0-or-later as a Secondary License; see the LICENSE file.
// SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later

#ifndef MC__MCOP_HPP
#define MC__MCOP_HPP

#include <stdexcept>
#include <vector>

namespace mc
{

//! @brief C++ structure to allow usage of MC++ types for DAG evaluation and as
//! template parameters in other MC++ types.
template <typename T>
struct Op
{
  // The primary template is a "not specialized" sentinel: every usable type
  // must provide a full specialization of mc::Op. Instantiating this template
  // means a specialization is missing. We fail at compile time (dependent on T,
  // so it only fires on actual instantiation) instead of recursing infinitely
  // at run time, e.g. diam(const T&x){ return diam(x); } -- which otherwise
  // overflows the stack/hangs.
  static_assert(sizeof(T) == 0,
                "mc::Op<T> is not specialized for this type; provide an mc::Op "
                "specialization.");
  static T
  point(const double c)
  {
    return T(c);
  }
  static T
  zeroone()
  {
    return T(0, 1);
  }
  static void
  I(T& x, const T& y)
  {
    x = y;
  }
  static double
  l(const T& x)
  {
    return x.l();
  }
  static double
  u(const T& x)
  {
    return x.u();
  }
  static double
  abs(const T& x)
  {
    return abs(x);
  }
  static double
  mid(const T& x)
  {
    return mid(x);
  }
  static double
  diam(const T& x)
  {
    return diam(x);
  }
  static T
  inv(const T& x)
  {
    return inv(x);
  }
  static T
  sqr(const T& x)
  {
    return sqr(x);
  }
  static T
  sqrt(const T& x)
  {
    return sqrt(x);
  }
  static T
  exp(const T& x)
  {
    return exp(x);
  }
  static T
  log(const T& x)
  {
    return log(x);
  }
  static T
  xlog(const T& x)
  {
    return xlog(x);
  }
  static T
  fabs(const T& x)
  {
    return fabs(x);
  }
  static T
  sin(const T& x)
  {
    return sin(x);
  }
  static T
  cos(const T& x)
  {
    return cos(x);
  }
  static T
  tan(const T& x)
  {
    return tan(x);
  }
  static T
  asin(const T& x)
  {
    return asin(x);
  }
  static T
  acos(const T& x)
  {
    return acos(x);
  }
  static T
  atan(const T& x)
  {
    return atan(x);
  }
  static T
  sinh(const T& x)
  {
    return sinh(x);
  }
  static T
  cosh(const T& x)
  {
    return cosh(x);
  }
  static T
  tanh(const T& x)
  {
    return tanh(x);
  }
  static T
  erf(const T& x)
  {
    return erf(x);
  }
  static T
  erfc(const T& x)
  {
    return erfc(x);
  }
  static T
  fstep(const T& x)
  {
    return fstep(x);
  }
  static T
  bstep(const T& x)
  {
    return bstep(x);
  }
  static T
  hull(const T& x, const T& y)
  {
    return hull(x, y);
  }
  static T
  min(const T& x, const T& y)
  {
    return min(x, y);
  }
  static T
  max(const T& x, const T& y)
  {
    return max(x, y);
  }
  template <typename X, typename Y>
  static T
  pow(const X& x, const Y& y)
  {
    return pow(x, y);
  }
  static T
  cheb(const T& x, const unsigned n)
  {
    return cheb(x, n);
  }
  static T
  prod(const unsigned n, const T* x)
  {
    return prod(n, x);
  }
  static T
  monom(const unsigned n, const T* x, const unsigned* k)
  {
    return monom(n, x, k);
  }
  static bool
  inter(T& xIy, const T& x, const T& y)
  {
    return inter(xIy, x, y);
  }
  static bool
  eq(const T& x, const T& y)
  {
    return x == y;
  }
  static bool
  ne(const T& x, const T& y)
  {
    return x != y;
  }
  static bool
  lt(const T& x, const T& y)
  {
    return x < y;
  }
  static bool
  le(const T& x, const T& y)
  {
    return x <= y;
  }
  static bool
  gt(const T& x, const T& y)
  {
    return x > y;
  }
  static bool
  ge(const T& x, const T& y)
  {
    return x >= y;
  }
};

}  // namespace mc

#include <cmath>

#include "mcfunc.hpp"

namespace mc
{

//! @brief Specialization of the structure mc::Op to allow usage of doubles as a
//! template parameter
template <>
struct Op<double>
{
  static double
  point(const double c)
  {
    return c;
  }
  static double
  zeroone()
  {
    throw std::runtime_error(
        "mc::Op<double>::zeroone -- function not overloaded");
  }
  static void
  I(double& x, const double& y)
  {
    x = y;
  }
  static double
  l(const double& x)
  {
    return x;
  }
  static double
  u(const double& x)
  {
    return x;
  }
  static double
  abs(const double& x)
  {
    return std::fabs(x);
  }
  static double
  mid(const double& x)
  {
    return x;
  }
  static double
  diam(const double& x)
  {
    return 0.;
  }
  static double
  inv(const double& x)
  {
    return mc::inv(x);
  }
  static double
  sqr(const double& x)
  {
    return mc::sqr(x);
  }
  static double
  sqrt(const double& x)
  {
    return std::sqrt(x);
  }
  static double
  exp(const double& x)
  {
    return std::exp(x);
  }
  static double
  log(const double& x)
  {
    return std::log(x);
  }
  static double
  xlog(const double& x)
  {
    return mc::xlog(x);
  }
  static double
  fabs(const double& x)
  {
    return std::fabs(x);
  }
  static double
  sin(const double& x)
  {
    return std::sin(x);
  }
  static double
  cos(const double& x)
  {
    return std::cos(x);
  }
  static double
  tan(const double& x)
  {
    return std::tan(x);
  }
  static double
  asin(const double& x)
  {
    return std::asin(x);
  }
  static double
  acos(const double& x)
  {
    return std::acos(x);
  }
  static double
  atan(const double& x)
  {
    return std::atan(x);
  }
  static double
  sinh(const double& x)
  {
    return std::sinh(x);
  }
  static double
  cosh(const double& x)
  {
    return std::cosh(x);
  }
  static double
  tanh(const double& x)
  {
    return std::tanh(x);
  }
  static double
  erf(const double& x)
  {
    return ::erf(x);
  }
  static double
  erfc(const double& x)
  {
    return ::erfc(x);
  }
  static double
  fstep(const double& x)
  {
    return mc::fstep(x);
  }
  static double
  bstep(const double& x)
  {
    return mc::bstep(x);
  }
  static double
  hull(const double& x, const double& y)
  {
    throw std::runtime_error("mc::Op<double>::hull -- function not overloaded");
  }
  static double
  min(const double& x, const double& y)
  {
    return std::min(x, y);
  }
  static double
  max(const double& x, const double& y)
  {
    return std::max(x, y);
  }
  static double
  cheb(const double& x, const unsigned n)
  {
    return mc::cheb(x, n);
  }
  template <typename X, typename Y>
  static double
  pow(const X& x, const Y& y)
  {
    return std::pow(x, y);
  }
  static double
  prod(const unsigned n, const double* x)
  {
    return mc::prod(n, x);
  }
  static double
  monom(const unsigned n, const double* x, const unsigned* k)
  {
    return mc::monom(n, x, k);
  }
  static bool
  inter(double& xIy, const double& x, const double& y)
  {
    xIy = x;
    return true;
  }
  static bool
  eq(const double& x, const double& y)
  {
    return x == y;
  }
  static bool
  ne(const double& x, const double& y)
  {
    return x != y;
  }
  static bool
  lt(const double& x, const double& y)
  {
    return x < y;
  }
  static bool
  le(const double& x, const double& y)
  {
    return x <= y;
  }
  static bool
  gt(const double& x, const double& y)
  {
    return x > y;
  }
  static bool
  ge(const double& x, const double& y)
  {
    return x >= y;
  }
};

}  // namespace mc
#endif
