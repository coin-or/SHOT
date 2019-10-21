// Copyright (C) 2009-2016 Benoit Chachuat, Imperial College London.
// All Rights Reserved.
// This code is published under the Eclipse Public License.

#ifndef MC__MCPROFIL_HPP
#define MC__MCPROFIL_HPP

#include "mcop.hpp"
#include "mcfunc.hpp"
#include <Interval.h>
#include <Functions.h>
#include <Constants.h>
namespace mc
{

//! @brief Specialization of the structure mc::Op for use of the type INTERVAL of <A href="http://www.ti3.tu-harburg.de/Software/PROFILEnglisch.html">PROFIL</A> as a template parameter in the other MC++ types
template <> struct Op< ::INTERVAL >
{
  typedef ::INTERVAL T;
  static T point( const double c ) { return T(c); }
  static T zeroone() { return T(0.,1.); }
  static void I(T& x, const T& y) { x = y; }
  static double l(const T& x) { return ::Inf(x); }
  static double u(const T& x) { return ::Sup(x); }
  static double abs (const T& x) { return ::Abs(x);  }
  static double mid (const T& x) { return ::Mid(x);  }
  static double diam(const T& x) { return ::Diam(x); }
  static T inv (const T& x) { if(::Inf(x)<=0. && ::Sup(x)>=0.) throw std::runtime_error("operation not permitted"); return T(1.)/x;  }
  static T sqr (const T& x) { return ::Sqr(x);  }
  static T sqrt(const T& x) { if(::Inf(x)<0.) throw std::runtime_error("operation not permitted"); return ::Sqrt(x); }
  static T log (const T& x) { if(::Inf(x)<=0.) throw std::runtime_error("operation not permitted"); return ::Log(x);  }
  static T xlog(const T& x) { return T( ::Pred(mc::xlog(mc::mid(::Inf(x),::Sup(x),std::exp(-1.)))), ::Succ(std::max(mc::xlog(::Inf(x)),mc::xlog(::Sup(x)))) ); }
  static T fabs(const T& x) { return T(::Pred(mc::mid(::Inf(x),::Sup(x),0.)),::Succ(::Abs(x))); }
  static T exp (const T& x) { return ::Exp(x);  }
  static T sin (const T& x) { return ::Sin(x);  }
  static T cos (const T& x) { return ::Cos(x);  }
  static T tan (const T& x) { return ::Tan(x);  }
  static T asin(const T& x) { return ::ArcSin(x); }
  static T acos(const T& x) { return ::ArcCos(x); }
  static T atan(const T& x) { return ::ArcTan(x); }
  static T erf (const T& x) { throw std::runtime_error("operation not permitted"); }
  static T erfc(const T& x) { throw std::runtime_error("operation not permitted"); }
  static T fstep(const T& x) { throw std::runtime_error("operation not permitted"); }
  static T bstep(const T& x) { throw std::runtime_error("operation not permitted"); }
  static T hull(const T& x, const T& y) { return ::Hull(x,y); }
  static T min (const T& x, const T& y) { return T( ::Pred(std::min(::Inf(x),::Inf(y))), ::Succ(std::min(::Sup(x),::Sup(y))) ); }
  static T max (const T& x, const T& y) { return T( ::Pred(std::max(::Inf(x),::Inf(y))), ::Succ(std::max(::Sup(x),::Sup(y))) ); }
  static T arh (const T& x, const double k) { return ::Exp(-x/k); }
  static T cheb (const T& x, const unsigned n) { return T(-1.,1.); }
  template <typename X> static T pow(const X& x, const int n) { return( (n>=3&&n%2)? ::Hull(::Power(Inf(x),n),::Power(Sup(x),n)): ::Power(x,n) ); }
  template <typename X, typename Y> static T pow(const X& x, const Y& y) { return ::Power(x,y); }
  static T monomial (const unsigned int n, const T* x, const int* k) { return n? ::Power(x[0], k[0]) * monomial( n-1, x+1, k+1 ): 1.; }
  //static bool inter(T& xIy, const T& x, const T& y) { return ::Intersection(xIy,x,y); }
  static bool inter(T& xIy, const T& x, const T& y) { if( ::Succ(::Sup(x)) < ::Pred(::Inf(y)) || ::Pred(::Inf(x)) > ::Succ(::Sup(y)) ) return false; xIy = T( ::Pred(std::max(::Inf(x),::Inf(y))), ::Succ(std::min(::Sup(x),::Sup(y))) ); return true; }
  static bool eq(const T& x, const T& y) { return x==y; }
  static bool ne(const T& x, const T& y) { return x!=y; }
  static bool lt(const T& x, const T& y) { return x<y;  }
  static bool le(const T& x, const T& y) { return x<=y; }
  static bool gt(const T& x, const T& y) { return y<x;  }
  static bool ge(const T& x, const T& y) { return y<=x; }
};

} // namespace mc

#endif
