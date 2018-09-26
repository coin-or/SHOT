// Copyright (C) 2009-2016 Benoit Chachuat, Imperial College London.
// All Rights Reserved.
// This code is published under the Eclipse Public License.

#ifndef MC__MCFUNC_HPP
#define MC__MCFUNC_HPP

#include <cmath>
#include <cfloat>

namespace mc
{

enum{ ICUT = 0, ICONV, ICONC };
const double PI = 4.0*std::atan(1.0);

inline double sign
( const double x )
{
  // Compute and return squared value
  return( x>=0? 1.: -1. );
}

inline double sqr
( const double x )
{
  // Return squared value
  return x*x;
}

inline int pow
( const int z, const unsigned n )
{
  // Return lower power value
  return n? z*pow(z,n-1): 1;
}

inline double cheb
( const double x, const unsigned n )
{
  switch( n ){
    case 0:  return 1.;
    case 1:  return x;
    case 2:  return 2.*x*x-1.;
    default: return 2.*x*cheb(x,n-1)-cheb(x,n-2);
    //default: return std::cos(n*std::acos(x));
  }
}

inline double cheb2
( const double x, const unsigned n )
{
  switch( n ){
    case 0:  return 1.;
    case 1:  return 2.*x;
    case 2:  return 4.*x*x-1.;
    default: return 2.*x*cheb2(x,n-1)-cheb2(x,n-2);
  }
}

inline double inv
( const double x )
{
  // Return inverse value
  return 1./x;
}

template <typename U> inline U bilin
( const U&x, const U&y, const unsigned int N=0 )
{
  // Return bilinear product
  return x*y;
}

inline double arh
( const double x, const double a )
{
  // Return arrehenius-like term
  return std::exp( - a / x );
}

inline double dsqr
( const double x )
{
  // CReturn derivatvie of squared value
  return 2.*x;
}

inline double xlog
( const double x )
{
  // Return x*log(x) term
  return x*std::log( x );
}

inline double monomial
(const unsigned int n, const double*x, const int*k)
{
  // Return monomial term \sum_i pow( x[i], k[i] )
  if( n == 0 ){
    return 1.;
  }
  if( n == 1 ){
    return std::pow( x[0], k[0] );
  }
  return std::pow( x[0], k[0] ) * monomial( n-1, x+1, k+1 );
}

inline double fstep
( const double x )
{
  // Return forward unit step at 0
  return ( x>=0? 1: 0. );
}

inline double bstep
( const double x )
{
  // Return backward unit step at 0
  return ( x>=0? 0: 1. );
}

inline double max
( const unsigned int n, const double*x )
{
  // Return maximum term \max_i { x[i] }
  if( n == 0 ) return 0.;
  if( n == 1 ) return x[0];
  return std::max( x[0], max( n-1, x+1 ));
}

inline double min
( const unsigned int n, const double*x )
{
  // Return minimum term \min_i { x[i] }
  if( n == 0 ) return 0.;
  if( n == 1 ) return x[0];
  return std::min( x[0], min( n-1, x+1 ));
}

inline unsigned int argmin
( const unsigned int n, const double*x )
{
  // Return argmin term \argmin_i { x[i] }
  if( n == 0 || !x ) return 0;
  unsigned int minndx = 0;
  double minval = x[0];
  for( unsigned int j=1; j<n; j++ )
    if( minval > x[j] ){ minval = x[j]; minndx = j; }
  return minndx;
}

inline unsigned int argmax
( const unsigned int n, const double*x )
{
  // Return argmax term \argmin_i { x[i] }
  if( n == 0 || !x ) return 0;
  unsigned int maxndx = 0;
  double maxval = x[0];
  for( unsigned int j=1; j<n; j++ )
    if( maxval < x[j] ){ maxval = x[j]; maxndx = j; }
  return maxndx;
}

inline double ltcond
( const double cond, const double expr1, const double expr2 )
{
  // Return condition expression { expr1 if cond<=0; expr2 else }
  return ( cond<=0? expr1: expr2 );
}

inline double gtcond
( const double cond, const double expr1, const double expr2 )
{
  // Return condition expression { expr1 if cond>=0; expr2 else }
  return ( cond>=0? expr1: expr2 );
}

inline double mid
( const double a, const double b, const double c )
{
  // Return the mid value of three scalars; e.g., b if a <= b <= c
  if(( b <= a && a <= c ) || ( c <= a && a <= b )) return a;
  if(( a <= b && b <= c ) || ( c <= b && b <= a )) return b;
  return c;
}

inline double mid
( const double CONV, const double CONC, const double CUT,
  int &id )
{
  // Return the mid value of three scalars CONV, CONC and CUT,
  // knowing that CONV <= CONC
  // If the argument <a>id</a> is unspecified (negative value),
  // the function indicates which value is the mid using the
  // enumeration enum{ ICUT = 0, ICONV, ICONC } defined abobe
  // in the mc namespace
  if( id < 0 ){
    if ( CUT < CONV )      { id = ICONV; return CONV; }
    else if ( CUT > CONC ) { id = ICONC; return CONC; }
    else                   { id = ICUT;  return CUT;  }
  }

  // If the argument <a>id</a> is specified (nonnegative value),
  // the function simply returns the corresponding value, which
  // is not necessarily the mid value
  if      ( id == ICONV ) return CONV;
  else if ( id == ICUT  ) return CUT;
  else                    return CONC;
}

inline double mid
( const double *DCONV, const double *DCONC, const int k, const int id )
{
  // Return the value corresponding to <a>id</a> if ICONV or ICONC;
  // otherwise return 0
  if      ( id == ICONV ) return DCONV[k];
  else if ( id == ICUT  ) return 0.;
  else                    return DCONC[k];
}

inline double mid
( const double *DCONV, const double *DCONC, const double *DCUT,
  const int k, const int id )
{
  // Return the value corresponding to <a>id</a>
  if      ( id == ICONV ) return DCONV[k];
  else if ( id == ICUT  ) return DCUT[k];
  else                    return DCONC[k];
}

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
isequal
( const double real1, const double real2, const double atol=machprec(),
  const double rtol=machprec() )
{
  // Test if two real values are within the same absolute and relative
  // tolerances
  double gap = std::fabs(real1-real2);
  double ave = 0.5*std::fabs(real1+real2);
  return( gap<atol+ave*rtol? true: false );
}

} // namespace mc
#endif
