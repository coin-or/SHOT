// Copyright (C) 2009-2016 Benoit Chachuat, Imperial College London.
// All Rights Reserved.
// This code is published under the Eclipse Public License.

#ifndef MC__MCOP_HPP
#define MC__MCOP_HPP

#include <stdexcept>

namespace mc
{

//! @brief C++ structure to allow usage of MC++ types for DAG evluation and as template parameters in other MC++ types.
template <typename T> struct Op
{
  static T point( const double c ) { return T(c); } // { throw std::runtime_error("mc::Op<T>::point -- Function not overloaded"); }
  static T zeroone() { return T(0,1); }
  static void I(T& x, const T& y) { x = y; }
  static double l(const T& x) { return x.l(); }
  static double u(const T& x) { return x.u(); }
  static double abs (const T& x) { return abs(x);  }
  static double mid (const T& x) { return mid(x);  }
  static double diam(const T& x) { return diam(x); }
  static T inv (const T& x) { return inv(x);  }
  static T sqr (const T& x) { return sqr(x);  }
  static T sqrt(const T& x) { return sqrt(x); }
  static T log (const T& x) { return log(x);  }
  static T xlog(const T& x) { return xlog(x); }
  static T fabs(const T& x) { return fabs(x); }
  static T exp (const T& x) { return exp(x);  }
  static T sin (const T& x) { return sin(x);  }
  static T cos (const T& x) { return cos(x);  }
  static T tan (const T& x) { return tan(x);  }
  static T asin(const T& x) { return asin(x); }
  static T acos(const T& x) { return acos(x); }
  static T atan(const T& x) { return atan(x); }
  static T erf (const T& x) { return erf(x);  }
  static T erfc(const T& x) { return erfc(x); }
  static T fstep(const T& x) { return fstep(x); }
  static T bstep(const T& x) { return bstep(x); }
  static T hull(const T& x, const T& y) { return hull(x,y); }
  static T min (const T& x, const T& y) { return min(x,y);  }
  static T max (const T& x, const T& y) { return max(x,y);  }
  static T arh (const T& x, const double k) { return arh(x,k); }
  static T cheb (const T& x, const unsigned n) { return cheb(x,n); }
  template <typename X, typename Y> static T pow(const X& x, const Y& y) { return pow(x,y); }
  static T monomial (const unsigned n, const T* x, const int* k) { return monomial(n,x,k); }
  static bool inter(T& xIy, const T& x, const T& y) { return inter(xIy,x,y); }
  static bool eq(const T& x, const T& y) { return x==y; }
  static bool ne(const T& x, const T& y) { return x!=y; }
  static bool lt(const T& x, const T& y) { return x<y;  }
  static bool le(const T& x, const T& y) { return x<=y; }
  static bool gt(const T& x, const T& y) { return x>y;  }
  static bool ge(const T& x, const T& y) { return x>=y; }
};

}

#include <cmath>
#include "mcfunc.hpp"

namespace mc
{

//! @brief Specialization of the structure mc::Op to allow usage of doubles as a template parameter
template <> struct Op< double >
{
  typedef double T;
  static T inv (const T& x) { return 1./x;  }
  static T sqr (const T& x) { return x*x;  }
  static T sqrt(const T& x) { return std::sqrt(x); }
  static T log (const T& x) { return std::log(x);  }
  static T xlog(const T& x) { return mc::xlog(x); }
  static T fabs(const T& x) { return std::fabs(x); }
  static T exp (const T& x) { return std::exp(x);  }
  static T sin (const T& x) { return std::sin(x);  }
  static T cos (const T& x) { return std::cos(x);  }
  static T tan (const T& x) { return std::tan(x);  }
  static T asin(const T& x) { return std::asin(x); }
  static T acos(const T& x) { return std::acos(x); }
  static T atan(const T& x) { return std::atan(x); }
  static T erf (const T& x) { return ::erf(x);  }
  static T erfc(const T& x) { return ::erfc(x); }
  static T fstep(const T& x) { return fstep(x); }
  static T bstep(const T& x) { return bstep(x); }
  static T hull(const T& x, const T& y) { throw std::runtime_error("mc::Op<double>::hull -- operation not permitted"); }
  static T min (const T& x, const T& y) { return min(x,y);  }
  static T max (const T& x, const T& y) { return max(x,y);  }
  static T arh (const T& x, const double k) { return mc::arh(x,k); }
  static T cheb (const T& x, const unsigned n) { return mc::cheb(x,n); }
  template <typename X, typename Y> static T pow(const X& x, const Y& y) { return std::pow(x,y); }
  static T monomial (const unsigned int n, const T* x, const int* k) { return mc::monomial(n,x,k); }
  static bool inter(T& xIy, const T& x, const T& y) { xIy = x; return true; }//{ throw std::runtime_error("mc::Op<double>::inter -- operation not permitted"); }
  static bool eq(const T& x, const T& y) { return x==y; }
  static bool ne(const T& x, const T& y) { return x!=y; }
  static bool lt(const T& x, const T& y) { return x<y;  }
  static bool le(const T& x, const T& y) { return x<=y; }
  static bool gt(const T& x, const T& y) { return x>y;  }
  static bool ge(const T& x, const T& y) { return x>=y; }
};

}
#endif
