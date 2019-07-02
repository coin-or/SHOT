// Copyright (C) 2009-2016 Benoit Chachuat, Imperial College London.
// All Rights Reserved.
// This code is published under the Eclipse Public License.

/*!
\page page_INTERVAL Non-Verified Interval Arithmetic for Factorable Functions
\author Beno&icirc;t Chachuat

Computational methods for enclosing the range of functions find their origins in interval analysis back in the early 1960s [Moore, 1966; Moore <I>et al.</I>, 2009]. For functions whose expressions can be broken down into a finite number of elementary unary and binary operations, namely factorable functions, interval bounding can be readily automated. The class mc::Interval provides a basic implementation of interval arithmetic, which is <B>not a verified implementation</B> in the sense that rounding errors are not accounted for. For verified interval computations, it is strongly recommended to use third-party libraries such as <A href="http://www.ti3.tu-harburg.de/Software/PROFILEnglisch.html">PROFIL</A> or <A href="http://www.math.uni-wuppertal.de/~xsc/software/filib.html">FILIB++</A>.

The implementation of mc::Interval relies on the operator/function overloading mechanism of C++. This makes the computation of bounds both simple and intuitive, similar to computing function values in real arithmetics. Moreover, mc::Interval can be used as the underlying interval type in other classes of MC++ via templates; e.g., mc::McCormick<mc::Interval>, mc::TModel<mc::Interval>, mc::TVar<mc::Interval>.


\section sec_INTERVAL_use How do I compute interval bounds on the range of a factorable function?

Suppose we want to calculate bounds on the range of the real-valued function \f$f(x,y)=x(\exp(x)-y)^2\f$ for \f$(x,y)\in [-2,1]^2\f$.

First, we shall define the variables \f$x\f$ and \f$y\f$. This is done as follows:

\code
      #include "interval.hpp"
      typedef mc::Interval I;

      I X( -2., 1. );
      I Y( -2., 1. );
\endcode

Essentially, the last two lines mean that <tt>X</tt> and <tt>Y</tt> are variable of type mc::Interval, both defined as \f$[-2,1]\f$.

Having defined the variables, bounds on the range of \f$f\f$ on \f$[-2,1]^2\f$ are simply calculated as

\code
      I F = X*pow(exp(X)-Y,2);
\endcode

These bounds can be displayed to the standard output as:

\code
      std::cout << "F bounds: " << F << std::endl;
\endcode

which produces the following output:

\verbatim
F bounds: [ -4.45244e+01 :  2.22622e+01 ] 
\endverbatim

Moreover, the upper and lower bounds in the interval bound <a>F</a> can be retrieved as:

\code
      double Fl = IF.l();
      double Fu = IF.u();
\endcode <tt>exp</tt>,


\section sec_INTERVAL_fct Which functions are overloaded in mc::Interval?

mc::Interval overloads the usual functions <tt>exp</tt>, <tt>log</tt>, <tt>sqr</tt>, <tt>sqrt</tt>, <tt>pow</tt>, <tt>inv</tt>, <tt>cos</tt>, <tt>sin</tt>, <tt>tan</tt>, <tt>acos</tt>, <tt>asin</tt>, <tt>atan</tt>, <tt>erf</tt>, <tt>erfc</tt>, <tt>min</tt>, <tt>max</tt>, <tt>fabs</tt>. mc::Interval also defines the following functions:
- <tt>inter(Z,X,Y)</tt>, computing the intersection \f$Z = X\cap Y\f$ and returning true/false if the intersection is nonempty/empty
- <tt>hull(X,Y)</tt>, returning the interval hull of \f$X\cup Y\f$
- <tt>diam(X)</tt>, returning the diameter of \f$X\f$
- <tt>mid(X)</tt>, returning the mid-point of \f$X\f$
- <tt>abs(X)</tt>, returning the absolute value of \f$X\f$
.


\section sec_INTERVAL_opt What are the options in mc::Interval and how are they set?

The class mc::Interval has a public static member called mc::Interval::options that can be used to set/modify the options; e.g.,

\code
      mc::Interval::options.DISPLAY_DIGITS = 7;
\endcode

The available options are as follows:

<TABLE border="1">
<CAPTION><EM>Options in mc::Interval::Options: name, type and description</EM></CAPTION>
     <TR><TH><b>Name</b>  <TD><b>Type</b><TD><b>Default</b>
         <TD><b>Description</b>
     <TR><TH><tt>DISPLAY_DIGITS</tt> <TD><tt>unsigned int</tt> <TD>5
         <TD>Number of digits in output stream
</TABLE>


\section sec_INTERVAL_err What errors can be encountered in using mc::Interval?

Errors are managed based on the exception handling mechanism of the C++ language. Each time an error is encountered, a class object of type Interval::Exceptions is thrown, which contains the type of error.

Possible errors encountered in using mc::Interval are:

<TABLE border="1">
<CAPTION><EM>Exceptions in mc::Interval</EM></CAPTION>
     <TR><TH><b>Number</b> <TD><b>Description</b>
     <TR><TH><tt>1</tt> <TD>Division by zero
     <TR><TH><tt>2</tt> <TD>Inverse with zero in range
     <TR><TH><tt>3</tt> <TD>Log with negative values in range
     <TR><TH><tt>4</tt> <TD>Square-root with nonpositive values in range
     <TR><TH><tt>5</tt> <TD>Inverse cosine with values outside of [-1,1] range
     <TR><TH><tt>6</tt> <TD>Inverse sine with values outside of [-1,1] range
     <TR><TH><tt>7</tt> <TD>Tangent with values \f$\frac{\pi}{2}+k\,\pi\f$, with \f$k\in\mathbb{Z}\f$, in range
</TABLE>

\section sec_INTERVAL_refs References

- Moore, R.E., <I><A href="http://books.google.co.uk/books/about/Interval_analysis.html?id=csQ-AAAAIAAJ&redir_esc=y2">"Interval Analysis"</A></I>, Prentice-Hall, 1966
- Moore, R.E., M.J. Cloud, R.B. Kearfott, <I><A href="http://books.google.co.uk/books/about/Introduction_to_interval_analysis.html?id=tT7ykKbqfEwC&redir_esc=y">"Introduction to Interval Analysis"</A></I>, SIAM, 2009
.

*/

#ifndef MC__INTERVAL_HPP
#define MC__INTERVAL_HPP

#include <iostream>
#include <iomanip>
#include <stdarg.h>

#include "mcfunc.hpp"

namespace mc
{
//! @brief C++ class for (non-verified) interval bounding of factorable function
////////////////////////////////////////////////////////////////////////
//! mc::Interval is a C++ class for interval bounding of factorable
//! functions on a box based on natural interval extensions. Round-off
//! errors are not accounted for in the computations (non-verified
//! implementation).
////////////////////////////////////////////////////////////////////////
class Interval
////////////////////////////////////////////////////////////////////////
{
  // friends of class Interval for operator overloading
  friend Interval operator+
    ( const Interval& );
  friend Interval operator+
    ( const Interval&, const Interval& );
  friend Interval operator+
    ( const double, const Interval& );
  friend Interval operator+
    ( const Interval&, const double );
  friend Interval operator-
    ( const Interval& );
  friend Interval operator-
    ( const Interval&, const Interval& );
  friend Interval operator-
    ( const double, const Interval& );
  friend Interval operator-
    ( const Interval&, const double );
  friend Interval operator*
    ( const Interval&, const Interval& );
  friend Interval operator*
    ( const Interval&, const double );
  friend Interval operator*
    ( const double, const Interval& );
  friend Interval operator/
    ( const Interval&, const Interval& );
  friend Interval operator/
    ( const Interval&, const double );
  friend Interval operator/
    ( const double, const Interval& );
  friend std::ostream& operator<<
    ( std::ostream&, const Interval& );
  friend bool operator==
    ( const Interval&, const Interval& );
  friend bool operator!=
    ( const Interval&, const Interval& );
  friend bool operator<=
    ( const Interval&, const Interval& );
  friend bool operator>=
    ( const Interval&, const Interval& );
  friend bool operator<
    ( const Interval&, const Interval& );
  friend bool operator>
    ( const Interval&, const Interval& );

  // friends of class Interval for function overloading
  friend double diam
    ( const Interval& );
  friend double abs
    ( const Interval& );
  friend double mid
    ( const Interval& );
  friend Interval inv
    ( const Interval& );
  friend Interval sqr
    ( const Interval& );
  friend Interval exp
    ( const Interval& );
  friend Interval log
    ( const Interval& );
  friend Interval cos
    ( const Interval& );
  friend Interval sin
    ( const Interval& );
  friend Interval tan
    ( const Interval& );
  friend Interval acos
    ( const Interval& );
  friend Interval asin
    ( const Interval& );
  friend Interval atan
    ( const Interval& );
  friend Interval fabs
    ( const Interval& );
  friend Interval sqrt
    ( const Interval& );
  friend Interval xlog
    ( const Interval& );
  friend Interval erf
    ( const Interval& );
  friend Interval erfc
    ( const Interval& );
  friend Interval fstep
    ( const Interval& );
  friend Interval bstep
    ( const Interval& );
  friend Interval arh
    ( const Interval&, const double );
  friend Interval pow
    ( const Interval&, const int );
  friend Interval pow
    ( const Interval&, const double );
  friend Interval pow
    ( const Interval&, const Interval& );
  friend Interval monomial
    ( const unsigned int, const Interval*, const int* );
  friend Interval cheb
    ( const Interval&, const unsigned );
  friend Interval hull
    ( const Interval&, const Interval& );
  friend Interval min
    ( const Interval&, const Interval& );
  friend Interval max
    ( const Interval&, const Interval& );
  friend Interval min
    ( const unsigned int, const Interval* );
  friend Interval max
    ( const unsigned int, const Interval* );
  friend bool inter
    ( Interval&, const Interval&, const Interval& );

public:

  // other operator overloadings (inline)
  Interval& operator=
    ( const double c )
    {
      _l = c;
      _u = c;
      return *this;
    }
  Interval& operator=
    ( const Interval&I )
    {
      _l = I._l;
      _u = I._u;
      return *this;
    }
  Interval& operator+=
    ( const double c )
    {
      _l += c;
      _u += c;
      return *this;
    }
  Interval& operator+=
    ( const Interval&I )
    {
      _l += I._l;
      _u += I._u;
      return *this;
    }
  Interval& operator-=
    ( const double c )
    {
      _l -= c;
      _u -= c;
      return *this;
    }
  Interval& operator-=
    ( const Interval&I )
    {
      Interval I2( _l, _u );
      _l = I2._l - I._u;
      _u = I2._u - I._l;
      return *this;
    }
  Interval& operator*=
    ( const double c )
    {
      Interval I2( _l, _u );
      *this = I2 * c;
      return *this;

      double t = _l;
      c>=0 ? _l*=c, _u*=c : _l=_u*c, _u=t*c;
      return *this;
    }
  Interval& operator*=
    ( const Interval&I )
    {
      Interval I2( _l, _u );
      *this = I2 * I;
      return *this;
    }
  Interval& operator/=
    ( const double c )
    {
      Interval I2( _l, _u );
      *this = I2 / c;
      return *this;
    }
  Interval& operator/=
    ( const Interval&I )
    {
      Interval I2( _l, _u );
      *this = I2 / I;
      return *this;
    }

  /** @defgroup INTERVAL Non-Validated Interval Arithmetic for Factorable Functions
   *  @{
   */
  //! @brief Options of mc::Interval
  static struct Options
  {
    //! @brief Constructor
    Options():
      DISPLAY_DIGITS(5)
      {}
    //! @brief Number of digits displayed with << operator (default=5)
    unsigned int DISPLAY_DIGITS;
  } options;

  //! @brief Exceptions of mc::Interval
  class Exceptions
  {
  public:
    //! @brief Enumeration type for mc::Interval exceptions
    enum TYPE{
      DIV=1,	//!< Division by zero
      INV,	//!< Inverse with zero in range
      LOG,	//!< Log with negative values in range
      SQRT,	//!< Square-root with nonpositive values in range
      ACOS,	//!< Inverse cosine with values outside of [-1,1] range
      ASIN,	//!< Inverse sine with values outside of [-1,1] range
      TAN,	//!< Tangent with values \f$\frac{\pi}{2}+k\,\pi\f$, with \f$k\in\mathbb{Z}\f$, in range
      CHEB	//!< Chebyshev basis function outside of [-1,1] range
    };
    //! @brief Constructor for error flag <a>ierr</a>
    Exceptions( TYPE ierr ) : _ierr( ierr ){}
    //! @brief Return error flag
    int ierr(){ return _ierr; }
    //! @brief Return error description
    std::string what(){
      switch( _ierr ){
      case DIV:
        return "mc::Interval\t Division by zero";
      case INV:
        return "mc::Interval\t Inverse with zero in range";
      case LOG:
        return "mc::Interval\t Log with negative values in range";
      case SQRT:
        return "mc::Interval\t Square-root with nonpositive values in range";
      case ACOS:
        return "mc::Interval\t Inverse cosine with values outside of [-1,1] range";
      case ASIN:
        return "mc::Interval\t Inverse sine with values outside of [-1,1] range";
      case TAN:
        return "mc::Interval\t Tangent with values pi/2+k*pi in range";
      case CHEB:
        return "mc::Interval\t Chebyshev basis outside of [-1,1] range";
      }
      return "mc::Interval\t Undocumented error";
    }

  private:
    TYPE _ierr;
  };
  //! @brief Default constructor (needed for arrays of mc::Interval elements)
  Interval()
    {}
  //! @brief Constructor for a constant value <a>c</a>
  Interval
    ( const double c ):
    _l(c), _u(c)
    {}
  //! @brief Constructor for a variable that belongs to the interval [<a>l</a>,<a>u</a>]
  Interval
    ( const double l, const double u ):
    _l(l<u?l:u), _u(l<u?u:l)
    {}
  //! @brief Copy constructor for the interval <a>I</a>
  Interval
    ( const Interval&I ):
    _l(I._l), _u(I._u)
    {}

  //! @brief Destructor
  ~Interval()
    {}

  //! @brief Return lower bound
  const double& l() const
    {
      return _l;
    }
  //! @brief Return upper bound
  const double& u() const
    {
      return _u;
    }

  //! @brief Set lower bound to <a>lb</a>
  void l ( const double lb )
    {
      _l = lb;
    }
  //! @brief Set upper bound to <a>ub</a>
  void u ( const double ub )
    {
      _u = ub;
    }
  /** @} */
  
private:

  //! @brief Lower bound
  double _l;
  //! @brief Upper bound
  double _u;
};

////////////////////////////////////////////////////////////////////////

inline Interval::Options Interval::options;

inline Interval
operator+
( const Interval&I )
{
  return I;
}

inline Interval
operator-
( const Interval&I )
{
  Interval I2( -I._u, -I._l );
  return I2;
}

inline Interval
operator+
( const double c, const Interval&I )
{
  Interval I2( c + I._l, c + I._u );
  return I2;
}

inline Interval
operator+
( const Interval&I, const double c )
{
  Interval I2( c + I._l, c + I._u );
  return I2;
}

inline Interval
operator+
( const Interval&I1, const Interval&I2 )
{
  Interval I3( I1._l+I2._l, I1._u+I2._u );
  return I3;
}

inline Interval
operator-
( const double c, const Interval&I )
{
  Interval I2( c - I._u, c - I._l );
  return I2;
}

inline Interval
operator-
( const Interval&I, const double c )
{
  Interval I2( I._l-c, I._u-c );
  return I2;
}

inline Interval
operator-
( const Interval&I1, const Interval&I2 )
{
  Interval I3( I1._l-I2._u, I1._u-I2._l );
  return I3;
}

inline Interval
operator*
( const double c, const Interval&I )
{
  Interval I2( c>=0? c*I._l: c*I._u, c>=0? c*I._u: c*I._l );
  return I2;
}

inline Interval
operator*
( const Interval&I, const double c )
{
  Interval I2( c>=0? c*I._l: c*I._u, c>=0? c*I._u: c*I._l );
  return I2;
}

inline Interval
operator*
( const Interval&I1, const Interval&I2 )
{
  Interval I3( std::min(std::min(I1._l*I2._l,I1._l*I2._u),
                        std::min(I1._u*I2._l,I1._u*I2._u)),
               std::max(std::max(I1._l*I2._l,I1._l*I2._u),
                        std::max(I1._u*I2._l,I1._u*I2._u)) );
  return I3;
}

inline Interval
operator/
( const Interval &I, const double c )
{
  if( isequal(c,0.) ) throw Interval::Exceptions( Interval::Exceptions::DIV );
  return (1./c)*I;
}

inline Interval
operator/
( const double c, const Interval&I )
{
  return c*inv(I);
}

inline Interval
operator/
( const Interval&I1, const Interval&I2 )
{
  return I1*inv(I2);
}

inline double
diam
( const Interval &I )
{
  return I._u-I._l;
}

inline double
mid
( const Interval &I )
{
  return 0.5*(I._u+I._l);
}

inline double
abs
( const Interval &I )
{
  return std::max(std::fabs(I._l),std::fabs(I._u));
}

inline Interval
inv
( const Interval &I )
{
  if ( I._l <= 0. && I._u >= 0. ) throw Interval::Exceptions( Interval::Exceptions::INV );
  Interval I2( 1./I._u, 1./I._l );
  return I2;
}

inline Interval
sqr
( const Interval&I )
{
  int imid = -1;
  return Interval( mc::sqr( mid(I._l,I._u,0.,imid) ),
                   std::max(mc::sqr(I._l),mc::sqr(I._u)) );
}

inline Interval
exp
( const Interval &I )
{
  return Interval( std::exp(I._l), std::exp(I._u) );
}

inline Interval
arh
( const Interval &I, const double a )
{
  return exp(-a/I);
}

inline Interval
log
( const Interval &I )
{
  if ( I._l <= 0. ) throw Interval::Exceptions( Interval::Exceptions::LOG );
  return Interval( std::log(I._l), std::log(I._u) );
}

inline Interval
xlog
( const Interval&I )
{
  if ( I._l <= 0. ) throw Interval::Exceptions( Interval::Exceptions::LOG );
  int imid = -1;
  return Interval( xlog(mid(I._l,I._u,std::exp(-1.),imid)),
                   std::max(xlog(I._l),xlog(I._u)) );
}

inline Interval
erf
( const Interval &I )
{
  return Interval( ::erf(I._l), ::erf(I._u) );
}

inline Interval
erfc
( const Interval &I )
{
  return Interval( ::erfc(I._u), ::erfc(I._l) );
}

inline Interval
sqrt
( const Interval&I )
{
  if ( I._l < 0. ) throw Interval::Exceptions( Interval::Exceptions::SQRT );
  return Interval( std::sqrt(I._l), std::sqrt(I._u) );
}

inline Interval
fabs
( const Interval&I )
{
  int imid = -1;
  return Interval( std::fabs(mid(I._l,I._u,0.,imid)),
                   std::max(std::fabs(I._l),std::fabs(I._u)) );
}

inline Interval
pow
( const Interval&I, const int n )
{
  if( n == 0 ){
    return 1.;
  }
  if( n == 1 ){
    return I;
  }
  if( n >= 2 && n%2 == 0 ){ 
    int imid = -1;
    return Interval( std::pow(mid(I._l,I._u,0.,imid),n),
                     std::max(std::pow(I._l,n),std::pow(I._u,n)) );
  }
  if ( n >= 3 ){
    return Interval( std::pow(I._l,n), std::pow(I._u,n) );
  }
  return inv( pow( I, -n ) );
}

inline Interval
monomial
(const unsigned int n, const Interval*I, const int*k)
{
  if( n == 0 ){
    return 1.;
  }
  if( n == 1 ){
    return pow( I[0], k[0] );
  }
  return pow( I[0], k[0] ) * monomial( n-1, I+1, k+1 );
}

inline Interval
cheb
( const Interval&I0, const unsigned n )
{
  Interval I(-1.,1.);
  if( !inter( I, I0, I ) ){
  //if ( I._l < -1.-1e1*machprec() || I._u > 1.+1e1*machprec() ){
    throw typename Interval::Exceptions( Interval::Exceptions::CHEB );
  }
  switch( n ){
    case 0:  return 1.;
    case 1:  return I;
    case 2:  return 2.*sqr(I)-1.;
    default:{
      int kL = n - std::ceil(n*std::acos(I._l)/mc::PI);  if( kL <= 0 ) kL = 0;
      int kU = n - std::floor(n*std::acos(I._u)/mc::PI); if( kU >= (int)n ) kU = n;
#ifdef MC__INTERVAL_CHEB_DEBUG
      std::cout << "  kL: " << kL << "  kU: " << kU;
#endif
      if( kU-kL <= 1 ){ // monotonic part
        double TL = std::cos(n*std::acos(I._l));
        double TU = std::cos(n*std::acos(I._u));
        return( TL<=TU? Interval(TL,TU): Interval(TU,TL) );
      }
      else if( kU-kL == 2 ){ // single extremum in range
        double TL = std::cos(n*std::acos(I._l));
        double TU = std::cos(n*std::acos(I._u));
        if( (n-kL)%2 ) return( TL<=TU? Interval(TL, 1.): Interval(TU, 1.) );  // minimum
        else           return( TL<=TU? Interval(-1.,TU): Interval(-1.,TL) );  // maximum
      }
      break;
    }
  }
  //Interval Icheb = 2.*I*cheb(I,n-1)-cheb(I,n-2);
  //return( inter( Icheb, Icheb, Interval(-1.,1.) )? Icheb: Interval(-1.,1.) );
  return Interval(-1.,1.);
}

inline Interval
pow
( const Interval&I, const double a )
{
  return exp( a * log( I ) );
}

inline Interval
pow
( const Interval&I1, const Interval&I2 )
{
  return exp( I2 * log( I1 ) );
}

inline Interval
hull
( const Interval&I1, const Interval&I2 )
{
  return Interval( std::min( I1._l, I2._l ), std::max( I1._u, I2._u ) );
}

inline Interval
min
( const Interval&I1, const Interval&I2 )
{
  return Interval( std::min( I1._l, I2._l ), std::min( I1._u, I2._u ) );
}

inline Interval
max
( const Interval&I1, const Interval&I2 )
{
  return Interval( std::max( I1._l, I2._l ), std::max( I1._u, I2._u ) );
}

inline Interval
min
( const unsigned int n, const Interval*I )
{
  Interval I2( n==0 || !I ? 0.: I[0] );
  for( unsigned int i=1; i<n; i++ ) I2 = min( I2, I[i] );
  return I2;
}

inline Interval
max
( const unsigned int n, const Interval*I )
{
  Interval I2( n==0 || !I ? 0.: I[0] );
  for( unsigned int i=1; i<n; i++ ) I2 = max( I2, I[i] );
  return I2;
}

inline Interval
cos
( const Interval&I )
{
  const int k = std::ceil(-(1.+I._l/PI)/2.); // -pi <= xL+2*k*pi < pi
  const double l = I._l+2.*PI*k, u = I._u+2.*PI*k;
  if( l <= 0 ){
    if( u <= 0 )   return Interval( std::cos(l), std::cos(u) );
    if( u >= PI )  return Interval( -1., 1. );
    return Interval( std::min(std::cos(l), std::cos(u)), 1. );
  }
  if( u <= PI )    return Interval( std::cos(u), std::cos(l) );
  if( u >= 2.*PI ) return Interval( -1., 1. );
  return Interval( -1., std::max(std::cos(l), std::cos(u)));
}

inline Interval
sin
( const Interval &I )
{
  return cos( I - PI/2. );
}

inline Interval
tan
( const Interval&I )
{
  const int k = std::ceil(-0.5-I._l/PI); // -pi/2 <= xL+k*pi < pi/2
  const double l = I._l+PI*k, u = I._u+PI*k;
  if( u >= 0.5*PI ) throw Interval::Exceptions( Interval::Exceptions::TAN );
  return Interval( std::tan(l), std::tan(u) );
}

inline Interval
acos
( const Interval &I )
{
  if ( I._l < -1. || I._u > 1. ) throw Interval::Exceptions( Interval::Exceptions::ACOS );
  return Interval( std::acos(I._u), std::acos(I._l) );
}

inline Interval
asin
( const Interval &I )
{
  if ( I._l < -1. || I._u > 1. ) throw Interval::Exceptions( Interval::Exceptions::ASIN );
  return Interval( std::asin(I._l), std::asin(I._u) );
}

inline Interval
atan
( const Interval &I )
{
  return Interval( std::atan(I._l), std::atan(I._u) );
}

inline Interval
fstep
( const Interval &I )
{
  if( I._l >= 0 )     return Interval(1.);
  else if( I._u < 0 ) return Interval(0.);
  return Interval(0.,1.);
}

inline Interval
bstep
( const Interval &I )
{
  return fstep( -I );
}

inline std::ostream&
operator<<
( std::ostream&out, const Interval&I)
{
  out << std::right << std::scientific << std::setprecision(Interval::options.DISPLAY_DIGITS);
  out << "[ "  << std::setw(Interval::options.DISPLAY_DIGITS+7) << I.l()
      << " : " << std::setw(Interval::options.DISPLAY_DIGITS+7) << I.u() << " ]";
  return out;
}

inline bool
inter
( Interval &XIY, const Interval &X, const Interval &Y )
{
  if( X._l > Y._u || Y._l > X._u ) return false;
  XIY._l = std::max( X._l, Y._l );
  XIY._u = std::min( X._u, Y._u );
  return true;
}

inline bool
operator==
( const Interval&I1, const Interval&I2 )
{
  return( I1._l == I2._l && I1._u == I2._u );
}

inline bool
operator!=
( const Interval&I1, const Interval&I2 )
{
  return( I1._l != I2._l || I1._u != I2._u );
}

inline bool
operator<=
( const Interval&I1, const Interval&I2 )
{
  return( I1._l >= I2._l && I1._u <= I2._u );
}

inline bool
operator>=
( const Interval&I1, const Interval&I2 )
{
  return( I1._l <= I2._l && I1._u >= I2._u );
}

inline bool
operator<
( const Interval&I1, const Interval&I2 )
{
  return( I1._l > I2._l && I1._u < I2._u );
}

inline bool
operator>
( const Interval&I1, const Interval&I2 )
{
  return( I1._l < I2._l && I1._u > I2._u );
}

} // namespace mc

#include "mcop.hpp"

namespace mc
{

//! @brief Specialization of the structure mc::Op to allow usage of the type mc::Interval for DAG evaluation or as a template parameter in other MC++ classes
template <> struct Op<mc::Interval>
{
  typedef mc::Interval T;
  static T point( const double c ) { return T(c); }
  static T zeroone() { return T(0.,1.); }
  static void I(T& x, const T&y) { x = y; }
  static double l(const T& x) { return x.l(); }
  static double u(const T& x) { return x.u(); }
  static double abs (const T& x) { return mc::abs(x);  }
  static double mid (const T& x) { return mc::mid(x);  }
  static double diam(const T& x) { return mc::diam(x); }
  static T inv (const T& x) { return mc::inv(x);  }
  static T sqr (const T& x) { return mc::sqr(x);  }
  static T sqrt(const T& x) { return mc::sqrt(x); }
  static T log (const T& x) { return mc::log(x);  }
  static T xlog(const T& x) { return mc::xlog(x); }
  static T fabs(const T& x) { return mc::fabs(x); }
  static T exp (const T& x) { return mc::exp(x);  }
  static T sin (const T& x) { return mc::sin(x);  }
  static T cos (const T& x) { return mc::cos(x);  }
  static T tan (const T& x) { return mc::tan(x);  }
  static T asin(const T& x) { return mc::asin(x); }
  static T acos(const T& x) { return mc::acos(x); }
  static T atan(const T& x) { return mc::atan(x); }
  static T erf (const T& x) { return mc::erf(x);  }
  static T erfc(const T& x) { return mc::erfc(x); }
  static T fstep(const T& x) { return mc::fstep(x); }
  static T bstep(const T& x) { return mc::bstep(x); }
  static T hull(const T& x, const T& y) { return mc::hull(x,y); }
  static T min (const T& x, const T& y) { return mc::min(x,y);  }
  static T max (const T& x, const T& y) { return mc::max(x,y);  }
  static T arh (const T& x, const double k) { return mc::arh(x,k); }
  static T cheb (const T& x, const unsigned n) { return mc::cheb(x,n); }
  template <typename X, typename Y> static T pow(const X& x, const Y& y) { return mc::pow(x,y); }
  static T monomial (const unsigned int n, const T* x, const int* k) { return mc::monomial(n,x,k); }
  static bool inter(T& xIy, const T& x, const T& y) { return mc::inter(xIy,x,y); }
  static bool eq(const T& x, const T& y) { return x==y; }
  static bool ne(const T& x, const T& y) { return x!=y; }
  static bool lt(const T& x, const T& y) { return x<y;  }
  static bool le(const T& x, const T& y) { return x<=y; }
  static bool gt(const T& x, const T& y) { return x>y;  }
  static bool ge(const T& x, const T& y) { return x>=y; }
};

} // namespace mc

#endif
