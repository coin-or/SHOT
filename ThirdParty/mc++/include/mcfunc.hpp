// Copyright (C) Benoit Chachuat, Imperial College London.
// All Rights Reserved.
// This code is published under the EPL-2.0 with GPL-2.0-or-later as a Secondary License; see the LICENSE file.
// SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later

#ifndef MC__MCFUNC_HPP
#define MC__MCFUNC_HPP

#include <cfloat>
#include <cmath>
#include <cstring>
#include <typeinfo>
#include <iostream>
#include <utility>
#include <vector>

namespace mc
{


//! @brief Whether two std::type_info objects denote the same type -- also across shared libraries (MC++ 5.0.4)
//!
//! An object created in one Python extension module (e.g. an FFEval built in pymcpp) and used in another (e.g. cronos,
//! built on MC++) carries the type_info of the module that created it: pybind11 modules hide their symbols, so each
//! has its own copy of the type_info of MC++'s classes.  libstdc++ (Linux) compares type_info by NAME, so `==` still
//! works; libc++ (macOS) compares by ADDRESS, so `typeid(FFVar) == idU` was false for an `idU` from the other module --
//! external operations then failed to recognise the arithmetic they were asked to evaluate in.  This compares with
//! `==` first, then by mangled name: exact type identity, as types with external linkage have unique mangled names.
inline bool
same_type
( std::type_info const& a, std::type_info const& b )
{
  if( a == b ) return true;
  char const* na = a.name();
  char const* nb = b.name();
  if( *na == '*' ) ++na;     // a leading '*' marks a name to be compared by address (GCC); not part of the name
  if( *nb == '*' ) ++nb;
  return std::strcmp( na, nb ) == 0;
}

//! @brief dynamic_cast to the EXACT class T -- also for an object created in another shared library (see same_type)
//!
//! dynamic_cast first (same module); otherwise, a static_cast when the dynamic type of *p is T by name.  Meant for
//! classes without subclasses, such as MC++'s external operations; both modules must be compiled from the same MC++
//! headers (same layout), which a pinned dependency guarantees.
template <typename T, typename B>
inline T*
type_cast
( B* p )
{
  if( !p ) return nullptr;
  if( auto* q = dynamic_cast<T*>( p ) ) return q;
  return same_type( typeid( *p ), typeid( T ) )? static_cast<T*>( p ): nullptr;
}
enum
{
  ICUT = 0,
  ICONV,
  ICONC
};
// const double PI = 4.0*std::atan(1.0);
const double PI = 3.14159265358979323846264338328;

inline unsigned
max(const unsigned x, const unsigned y)
{
  // Compute and return max value
  return (x >= y ? x : y);
}

inline unsigned
min(const unsigned x, const unsigned y)
{
  // Compute and return max value
  return (x >= y ? y : x);
}

inline double
sign(const double x)
{
  // Compute and return squared value
  return (x >= 0 ? 1. : -1.);
}

inline double
sqr(const double x)
{
  // Return squared value
  return x * x;
}

inline int
pow(const int z, const unsigned n)
{
  // Return lower power value
  return n ? z * pow(z, n - 1) : 1;
}

/*
inline std::vector<double> chebcoef
( const unsigned n )
{
  if( n == 0 ){
    std::vector<double> coef(1);
    coef[0]=1;
  }
  else if( n == 1 ){
    std::vector<double> coef(2);
    coef[0]=1;
    coef[1]=0;
  }
  else{
    auto&& coef = chebcoef(n-1);                                     //
generation of cheb(x,n-1) for( unsigned i=0; i<coef.size(); i++ ) coef[i] *= 2;
// multiplication by 2 coef.push_back(0.); // multiplication by x auto&& coef2 =
chebcoef(n-2);                                    // generation of cheb(x,n-2)
    for( unsigned i=0; i<coef2.size(); i++ ) coef[i+1] -= coef2[i];  //
subtraction of cheb(x,n-2)
  }
  return coef;
}
*/

inline std::vector<double>
chebcoef(const unsigned n)
{
  std::vector<double> coef2(1);
  coef2[0] = 1;
  if (n == 0) return coef2;

  std::vector<double> coef1(2);
  coef1[0] = 1;
  coef1[1] = 0;
  if (n == 1) return coef1;

  std::vector<double> coef;
  for (unsigned k = 1; k < n; k++)
  {
    coef = coef1;  // local copy of cheb(x,n-1)
    for (unsigned i = 0; i < coef.size(); i++)
      coef[i] *= 2;      // multiplication by 2
    coef.push_back(0.);  // multiplication by x
    for (unsigned i = 0; i < coef2.size(); i++)
      coef[i + 2] -= coef2[i];  // subtraction of cheb(x,n-2)
    std::swap(coef1, coef2);    // move cheb(x,n-1) to cheb(x,n-2)
    std::swap(coef, coef1);     // move cheb(x,n) to cheb(x,n-1)
  }
  return coef1;
}

inline double
cheb(const double x, const unsigned n)
{
  switch (n)
  {
    case 0:
      return 1.;
    case 1:
      return x;
    case 2:
      return 2. * x * x - 1.;
    default:
      return 2. * x * cheb(x, n - 1) - cheb(x, n - 2);
      // default: return std::cos(n*std::acos(x));
  }
}

inline double
cheb2(const double x, const unsigned n)
{
  switch (n)
  {
    case 0:
      return 1.;
    case 1:
      return 2. * x;
    case 2:
      return 4. * x * x - 1.;
    default:
      return 2. * x * cheb2(x, n - 1) - cheb2(x, n - 2);
  }
}

inline double
inv(const double x)
{
  // Return inverse value
  return 1. / x;
}
/*
template <typename U> inline U bilin
( const U&x, const U&y, const unsigned int N=0 )
{
  // Return bilinear product
  return x*y;
}
*/
inline double
arrh(const double x, const double a)
{
  // Return arrhenius-like term
  return std::exp(-a / x);
}

inline double
dsqr(const double x)
{
  // Return derivative of squared value
  return 2. * x;
}

inline double
fstep(const double x)
{
  // Return forward unit step at 0
  return (x >= 0 ? 1 : 0.);
}

inline double
bstep(const double x)
{
  // Return backward unit step at 0
  return (x >= 0 ? 0 : 1.);
}

inline double
prod(const unsigned int n, const double* x)
{
  // Return product term \prod_i x[i]
  // std::cout << "prod(" << n << ",x)" << std::endl;
  switch (n)
  {
    case 0:
      return 1.;
    case 1:
      return x[0];
    default:
      return x[0] * prod(n - 1, x + 1);
  }
}

inline double
monom(const unsigned int n, const double* x, const unsigned* k)
{
  // Return monomial term \prod_i pow( x[i], k[i] )
  switch (n)
  {
    case 0:
      return 1.;
    case 1:
      return std::pow(x[0], k[0]);
    default:
      return std::pow(x[0], k[0]) * monom(n - 1, x + 1, k + 1);
  }
}

inline double
max(const unsigned int n, const double* x)
{
  // Return maximum term \max_i { x[i] }
  switch (n)
  {
    case 0:
      return 0.;
    case 1:
      return x[0];
    default:
      return std::max(x[0], max(n - 1, x + 1));
  }
}

inline double
min(const unsigned int n, const double* x)
{
  // Return minimum term \min_i { x[i] }
  switch (n)
  {
    case 0:
      return 0.;
    case 1:
      return x[0];
    default:
      return std::min(x[0], min(n - 1, x + 1));
  }
}

inline unsigned int
argmin(const unsigned int n, const double* x)
{
  // Return argmin term \argmin_i { x[i] }
  if (n == 0 || !x) return 0;
  unsigned int minndx = 0;
  double minval       = x[0];
  for (unsigned int j = 1; j < n; j++)
    if (minval > x[j])
    {
      minval = x[j];
      minndx = j;
    }
  return minndx;
}

inline unsigned int
argmax(const unsigned int n, const double* x)
{
  // Return argmax term \argmin_i { x[i] }
  if (n == 0 || !x) return 0;
  unsigned int maxndx = 0;
  double maxval       = x[0];
  for (unsigned int j = 1; j < n; j++)
    if (maxval < x[j])
    {
      maxval = x[j];
      maxndx = j;
    }
  return maxndx;
}

inline double
ltcond(const double cond, const double expr1, const double expr2)
{
  // Return condition expression { expr1 if cond<=0; expr2 else }
  return (cond <= 0 ? expr1 : expr2);
}

inline double
gtcond(const double cond, const double expr1, const double expr2)
{
  // Return condition expression { expr1 if cond>=0; expr2 else }
  return (cond >= 0 ? expr1 : expr2);
}

inline double
mid(const double a, const double b, const double c)
{
  // Return the mid value of three scalars; e.g., b if a <= b <= c
  if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
  if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
  return c;
}

inline std::pair<double, double>
extreme(const double& a, const double& b, const double& c)
{
  if (a <= b && a <= c) return std::make_pair(a, b >= c ? b : c);
  if (b <= c) return std::make_pair(b, a >= c ? a : c);
  return std::make_pair(c, a >= b ? a : b);
}

inline double
mid(const double CONV, const double CONC, const double CUT, int& id)
{
  // Return the mid value of three scalars CONV, CONC and CUT,
  // knowing that CONV <= CONC
  // If the argument <a>id</a> is unspecified (negative value),
  // the function indicates which value is the mid using the
  // enumeration enum{ ICUT = 0, ICONV, ICONC } defined abobe
  // in the mc namespace
  if (id < 0)
  {
    // AVT.SVT 15.03.2017 change: made inequality signs < and > to lessequal and
    // greaterequal to avoid setting id=ICUT=0 falsely, e.g., with < and > the
    // subgradient of log(x) over X=[0.1,2] is returned as 0 at 0.1 which is
    // maybe not wrong, since it is a valid subgradient but it is not what the
    // user wants/expects
    if (CUT <= CONV)
    {
      id = ICONV;
      return CONV;
    }
    else if (CUT >= CONC)
    {
      id = ICONC;
      return CONC;
    }
    else
    {
      id = ICUT;
      return CUT;
    }
  }

  // If the argument <a>id</a> is specified (nonnegative value),
  // the function simply returns the corresponding value, which
  // is not necessarily the mid value
  if (id == ICONV)
    return CONV;
  else if (id == ICUT)
    return CUT;
  else
    return CONC;
}

// added AVT.SVT 31.05.2017: had to add the old function for non differentiable
// relaxations
//                           such as max, min..., since we cannot directly
//                           determine the derivative at border points
inline double
mid_ndiff(const double CONV, const double CONC, const double CUT, int& id)
{
  // Return the mid value of three scalars CONV, CONC and CUT,
  // knowing that CONV <= CONC
  // If the argument <a>id</a> is unspecified (negative value),
  // the function indicates which value is the mid using the
  // enumeration enum{ ICUT = 0, ICONV, ICONC } defined abobe
  // in the mc namespace
  if (id < 0)
  {
    if (CUT < CONV)
    {
      id = ICONV;
      return CONV;
    }
    else if (CUT > CONC)
    {
      id = ICONC;
      return CONC;
    }
    else
    {
      id = ICUT;
      return CUT;
    }
  }

  // If the argument <a>id</a> is specified (nonnegative value),
  // the function simply returns the corresponding value, which
  // is not necessarily the mid value
  if (id == ICONV)
    return CONV;
  else if (id == ICUT)
    return CUT;
  else
    return CONC;
}

inline double
mid(const double* DCONV, const double* DCONC, const int k, const int id)
{
  // Return the value corresponding to <a>id</a> if ICONV or ICONC;
  // otherwise return 0
  if (id == ICONV)
    return DCONV[k];
  else if (id == ICUT)
    return 0.;
  else
    return DCONC[k];
}

inline double
mid(const double* DCONV, const double* DCONC, const double* DCUT, const int k,
    const int id)
{
  // Return the value corresponding to <a>id</a>
  if (id == ICONV)
    return DCONV[k];
  else if (id == ICUT)
    return DCUT[k];
  else
    return DCONC[k];
}

//! @brief Function returning the machine precision for double precision
//! floating point number
inline double
machprec()
{
  return DBL_EPSILON;
  /*
    static bool   BTINY = false;
    static double RTINY = 1.0;
    if( BTINY ) return RTINY;

    double RONE = 1.0;
    int cnt = 0;
    while ( RONE + RTINY > RONE && cnt < 64 ){
      RTINY /= 2.;
      cnt++;
    }
    RTINY *= 2.;
    BTINY = true;
    return RTINY;
  */
}

inline bool
isequal(const double real1, const double real2, const double atol = DBL_MIN,
        const double rtol = DBL_EPSILON)
{
  // Test if two real values are within the same absolute and relative
  // tolerances
  double gap = std::fabs(real1 - real2);
  double ave = 0.5 * std::fabs(real1 + real2);
  return (gap > atol + ave * rtol ? false : true);
}

inline double
relu(const double x)
{
  // Return ReLU value
  return (x > 0 ? x : 0.);
}

inline double
xlog(const double x)
{
  if (isequal(x, 0.)) return 0.;
  // Return x*log(x) term
  return x * std::log(x);
}

inline double
lmtd(const double x, const double y)
{
  if (isequal(x, y)) return x;
  // Return lmtd(x,y) term
  return (x - y) / (std::log(x) - std::log(y));
}

inline double
rlmtd(const double x, const double y)
{
  if (isequal(x, y)) return 1. / x;
  // Return lmtd(x,y) term
  return (std::log(x) - std::log(y)) / (x - y);
}

}  // namespace mc

#endif
