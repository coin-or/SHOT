// Copyright (C) 2009-2016 Benoit Chachuat, Imperial College London.
// All Rights Reserved.
// This code is published under the Eclipse Public License.

/*!
\page page_FFDEP Structure and Dependency Detection for Factorable Functions
\author Benoit C. Chachuat
\version 0.1
\date 2011
\bug No known bugs.

mc::FFDep is a C++ class that determines the structure of mathematical expressions, namely their sparsity pattern and linearity, for a given set of participating variables. It relies on the operator overloading and function overloading mechanisms of C++. The overloaded operators are: `+', `-', `*', and `/'; the overloaded functions are: `exp', `log', `sqr', `pow', `sqrt', `fabs', `xlog', `min', `max', `cos', `sin', `tan', `acos', `asin' and `atan'.


\section sec_FFDepEval How Do I Determine the Structure of a Factorable Function?

Suppose you are given 4 variables \f$x_1,\ldots,x_4\f$ and want to determine the sparsity pattern and linearity of the vector following function
\f{eqnarray*}
  {\bf f}({\bf x}) = \left(\begin{array}{c} f_1({\bf x})\\ f_2({\bf x})\end{array}\right) = \left(\begin{array}{c} x_3 x_4+x_1\\x_1(\exp(x_3-x_4))^2+x_2 \end{array}\right)
\f}

First, define the variables \f$x_1,\ldots,x_4\f$ as

\code
      const int NX = 4;
      FFDep X[NX];
      for( int i=0; i<NX; i++ ) X[i].indep(i);
\endcode

Essentially, the first line means that <tt>X</tt> is an array of mc::FFDep class objects, and the second line defines X[0],...X[NX-1] as independent variables with indices 0,...,NX-1, respectively.

Once the independent variables \f${\bf x}\f$ have been defined, determine the structure of \f${\bf f}({\bf x})\f$ simply as

\code
      const int NF = 2;
      FFDep F[NF] = { X[2]*X[3]+X[0],
                      X[0]*pow(exp(X[2]-X[3]),2)+X[1] };
\endcode

Retrieve the structure - both the sparsity pattern and the linearity - of \f$f_1\f$ and \f$f_2\f$ as

\code
      std::map<int,bool> F0_dep = F[0].dep();
      std::map<int,bool> F1_dep = F[1].dep();
\endcode

You can also display the structure as

\code
      std::cout << "Variable dependence of F[0]: " << F[0] << std::endl;
      std::cout << "Variable dependence of F[1]: " << F[1] << std::endl;
\endcode

The corresponding output is

\verbatim
      Variable dependence of F[0]: { 0L 2NL 3NL }
      Variable dependence of F[1]: { 0NL 1L 2NL 3NL }
\endverbatim

which indicates that X[0], X[2] and X[3] participate in F[0], but not X[1]. Moreover, X[0] participates linearly, unlike X[2] and X[3].

\section sec_FFDepErr Errors Encountered in Determining the Structure of a Factorable Function?

Errors are managed based on the exception handling mechanism of the C++ language. Each time an error is encountered, a class object of type FFDep::Exceptions is thrown, which contains the type of error. It is the user's responsibility to test whether an exception was thrown during a calculation, and then make the appropriate changes. Should an exception be thrown and not caught by the calling program, the execution will stop.

Possible errors encountered in determining the structure of a factorable function are:

<TABLE border="1">
<CAPTION><EM>Errors during Structure Determination</EM></CAPTION>
     <TR><TH><b>Number</b> <TD><b>Description</b>
     <TR><TH><tt>-33</tt> <TD>Error due to calling a feature not yet implemented in mc::FFDep
</TABLE>

*/

#ifndef MC__FFDEP_HPP
#define MC__FFDEP_HPP

#include <iostream>
#include <map>

namespace mc
{

//! @brief C++ class for evaluation of the sparsity pattern of a factorable function
////////////////////////////////////////////////////////////////////////
//! mc::FFDep is a C++ class for evaluating the sparsity pattern of a
//! factorable function
////////////////////////////////////////////////////////////////////////
class FFDep
////////////////////////////////////////////////////////////////////////
{
  // friends of class FFDep for operator and function overloading
  friend FFDep operator+  ( const FFDep& );
  friend FFDep operator+  ( const FFDep&, const FFDep& );
  friend FFDep operator+  ( const double, const FFDep& );
  friend FFDep operator+  ( const FFDep&, const double );
  friend FFDep operator-  ( const FFDep& );
  friend FFDep operator-  ( const FFDep&, const FFDep& );
  friend FFDep operator-  ( const double, const FFDep& );
  friend FFDep operator-  ( const FFDep&, const double );
  friend FFDep operator*  ( const FFDep&, const FFDep& );
  friend FFDep operator*  ( const FFDep&, const double );
  friend FFDep operator*  ( const double, const FFDep& );
  friend FFDep operator/  ( const FFDep&, const FFDep& );
  friend FFDep operator/  ( const FFDep&, const double );
  friend FFDep operator/  ( const double, const FFDep& );
  friend std::ostream& operator<< ( std::ostream&, const FFDep& );
  friend bool operator==  ( const FFDep&, const FFDep& );
  friend bool operator!=  ( const FFDep&, const FFDep& );
  friend bool operator<=  ( const FFDep&, const FFDep& );
  friend bool operator>=  ( const FFDep&, const FFDep& );
  friend bool operator<   ( const FFDep&, const FFDep& );
  friend bool operator>   ( const FFDep&, const FFDep& );
  friend FFDep inv   ( const FFDep& );
  friend FFDep sqr   ( const FFDep& );
  friend FFDep exp   ( const FFDep& );
  friend FFDep log   ( const FFDep& );
  friend FFDep cos   ( const FFDep& );
  friend FFDep sin   ( const FFDep& );
  friend FFDep tan   ( const FFDep& );
  friend FFDep acos  ( const FFDep& );
  friend FFDep asin  ( const FFDep& );
  friend FFDep atan  ( const FFDep& );
  friend FFDep fabs  ( const FFDep& );
  friend FFDep sqrt  ( const FFDep& );
  friend FFDep erf   ( const FFDep& );
  friend FFDep erfc  ( const FFDep& );
  friend FFDep fstep ( const FFDep& );
  friend FFDep bstep ( const FFDep& );
  friend FFDep cheb  ( const FFDep&, const unsigned );
  friend FFDep pow   ( const FFDep&, const int );
  friend FFDep pow   ( const FFDep&, const double );
  friend FFDep pow   ( const FFDep&, const FFDep& );
  friend FFDep min   ( const FFDep&, const FFDep& );
  friend FFDep max   ( const FFDep&, const FFDep& );
  friend FFDep inter ( const FFDep&, const FFDep& );
  friend FFDep min   ( const unsigned int, const FFDep* );
  friend FFDep max   ( const unsigned int, const FFDep* );
  friend FFDep sum   ( const unsigned int, const FFDep* );
  friend FFDep prod  ( const unsigned int, const FFDep* );

public:

  //! @brief Exceptions of mc::FFDep
  class Exceptions
  {
  public:
    //! @brief Enumeration type for FFDep exception handling
    enum TYPE{
      UNDEF=-33	//!< Error due to calling a function/feature not yet implemented in mc::FFDep
    };
    //! @brief Constructor for error <a>ierr</a>
    Exceptions( TYPE ierr ) : _ierr( ierr ){}

    //! @brief Inline function returning the error flag
    int ierr(){ return _ierr; }
  private:
    TYPE _ierr;
    //! @brief Error description
    std::string what(){
      switch( _ierr ){
      case UNDEF:
        return "mc::FFDep\t Feature not yet implemented in MC++";
      default:
        return "mc::FFDep\t Undocumented error";
      }
    }
  };

  typedef std::map<int,bool> t_FFDep;
  typedef t_FFDep::iterator it_FFDep;
  typedef t_FFDep::const_iterator cit_FFDep;

  // other operator overloadings (inlined)
  FFDep& operator=
    ( const double c )
    { _dep.clear(); return *this; }
  FFDep& operator=
    ( const FFDep&S )
    { if( this != &S ) _dep = S._dep; return *this; }
  FFDep& operator+=
    ( const double c )
    { return *this; }
  FFDep& operator+=
    ( const FFDep&S )
    { return combine( S ); }
  FFDep& operator-=
    ( const double c )
    { return *this; }
  FFDep& operator-=
    ( const FFDep&S )
    { return combine( S ); }
  FFDep& operator*=
    ( const double c )
    { return *this; }
  FFDep& operator*=
    ( const FFDep&S )
    { return combine( S, false ); }
  FFDep& operator/=
    ( const double c )
    { return *this; }
  FFDep& operator/=
    ( const FFDep&S )
    { return combine( S, false ); }

  /** @defgroup FFDep Structure and Dependency Detection for Factorable Functions
   *  @{
   */
  //! @brief Default constructor (needed to declare arrays of FFDep class)
  FFDep
    ( const double c=0. )
    {}
  //! @brief Copy constructor
  FFDep
    ( const FFDep&S ):
    _dep(S._dep)
    {}
  //! @brief Destructor
  ~FFDep()
    {}

  //! @brief Sets as independent with index <a>ind</a>
  FFDep& indep
    ( const int ind )
    { _dep.clear(); _dep.insert( std::make_pair(ind,true) ); return *this; }

  //! @brief Determines if the current object is dependent on the variable of index <a>ind</a>
  std::pair<bool,bool> dep
    ( const int ind )
    { cit_FFDep it = _dep.find(ind);
      return( it==_dep.end()? std::make_pair(false,true): std::make_pair(true,it->second) ); }

  //! @brief Returns the dependency set
  const t_FFDep& dep() const
    { return _dep; }
  t_FFDep& dep()
    { return _dep; }

  //! @brief Combines with the dependency sets of another variable
  FFDep& combine
    ( const FFDep&S, const bool linear=true );
  //! @brief Combines the dependency sets of two variables
  static FFDep combine
    ( const FFDep&S1, const FFDep&S2, const bool linear=true );

  //! @brief Turns current dependent variables into nonlinear
  FFDep& nonlinear();
  //! @brief Turns current dependent variables into nonlinear
  static FFDep nonlinear
    ( const FFDep&S );
  /** @} */
  
private:

  //! @brief Dependency set
  t_FFDep _dep;
};

////////////////////////////////////////////////////////////////////////

inline FFDep&
FFDep::nonlinear()
{
  it_FFDep it = _dep.begin();
  for( ; it != _dep.end(); ++it ) it->second = false;
  return *this;
}

inline FFDep
FFDep::nonlinear
( const FFDep&S )
{
  FFDep S2( S );
  return S2.nonlinear(); 
}

inline FFDep&
FFDep::combine
( const FFDep&S, const bool linear )
{
  cit_FFDep cit = S._dep.begin();
  for( ; cit != S._dep.end(); ++cit ){
    std::pair<it_FFDep,bool> ins = _dep.insert( *cit );
    if( !ins.second ) ins.first->second = ( ins.first->second && cit->second );
  }
  return( linear? *this: nonlinear() );
}

inline FFDep
FFDep::combine
( const FFDep&S1, const FFDep&S2, const bool linear )
{
  FFDep S3( S1 );
  return S3.combine( S2, linear );
}

inline std::ostream&
operator<<
( std::ostream&out, const FFDep&S)
{
  out << "{ ";
  FFDep::cit_FFDep iS = S._dep.begin();
  for( ; iS != S._dep.end(); ++iS )
    out << iS->first << (iS->second?"L":"NL") << " ";
  out << "}";
  return out;
}

inline FFDep
operator+
( const FFDep&S )
{
  return S;
}

inline FFDep
operator-
( const FFDep&S )
{
  return S;
}

inline FFDep
operator+
( const double c, const FFDep&S )
{
  return S;
}

inline FFDep
operator+
( const FFDep&S, const double c )
{
  return S;
}

inline FFDep
operator+
( const FFDep&S1, const FFDep&S2 )
{ 
  return FFDep::combine( S1, S2 );
}

inline FFDep
sum
( const unsigned int n, const FFDep*S )
{
  if( n==2 ) return S[0] + S[1];
  return S[0] + sum( n-1, S+1 );
}

inline FFDep
operator-
( const double c, const FFDep&S )
{
  return S;
}

inline FFDep
operator-
( const FFDep&S, const double c )
{
  return S;
}

inline FFDep
operator-
( const FFDep&S1, const FFDep&S2 )
{
  return FFDep::combine( S1, S2 );
}

inline FFDep
operator*
( const double c, const FFDep&S )
{
  return S;
}

inline FFDep
operator*
( const FFDep&S, const double c )
{
  return S;
}

inline FFDep
operator*
( const FFDep&S1, const FFDep&S2 )
{
  if( S1._dep.empty() ) return S2;
  if( S2._dep.empty() ) return S1;
  return FFDep::combine( S1, S2, false );
}

inline FFDep
prod
( const unsigned int n, const FFDep*S )
{
  if( n==2 ) return S[0] * S[1];
  return S[0] * prod( n-1, S+1 );
}

inline FFDep
operator/
( const FFDep&S, const double c )
{
  return S;
}

inline FFDep
operator/
( const double c, const FFDep&S )
{
  return S;
}

inline FFDep
operator/
( const FFDep&S1, const FFDep&S2 )
{
  if( S1._dep.empty() ) return inv( S2 );
  if( S2._dep.empty() ) return S1;
  return FFDep::combine( S1, S2, false );
}

inline FFDep
inv
( const FFDep &S )
{
  return FFDep::nonlinear( S );
}

inline FFDep
sqr
( const FFDep&S )
{
  return FFDep::nonlinear( S );
}

inline FFDep
exp
( const FFDep &S )
{
  return FFDep::nonlinear( S );
}

inline FFDep
arh
( const FFDep &S, const double a )
{
  return FFDep::nonlinear( S );
}

inline FFDep
log
( const FFDep &S )
{
  return FFDep::nonlinear( S );
}

inline FFDep
xlog
( const FFDep&S )
{
  return FFDep::nonlinear( S );
}

inline FFDep
erf
( const FFDep &S )
{
  return FFDep::nonlinear( S );
}

inline FFDep
erfc
( const FFDep &S )
{
  return FFDep::nonlinear( S );
}

inline FFDep
fstep
( const FFDep &S )
{
  return FFDep::nonlinear( S );
}

inline FFDep
bstep
( const FFDep &S )
{
  return FFDep::nonlinear( S );
}

inline FFDep
sqrt
( const FFDep&S )
{
  return FFDep::nonlinear( S );
}

inline FFDep
fabs
( const FFDep&S )
{
  return FFDep::nonlinear( S );
}

inline FFDep
cheb
( const FFDep&S, const unsigned n )
{
  if( n == 0 ){ FFDep S2; return S2; }
  if( n == 1 ) return S;
  return FFDep::nonlinear( S );
}

inline FFDep
pow
( const FFDep&S, const int n )
{
  if( n == 0 ){ FFDep S2; return S2; }
  if( n == 1 ) return S;
  return FFDep::nonlinear( S );
}

inline FFDep
pow
( const FFDep&S, const double a )
{
  return FFDep::nonlinear( S );
}

inline FFDep
pow
( const FFDep&S1, const FFDep&S2 )
{
  if( S1._dep.empty() ) return FFDep::nonlinear( S2 );
  if( S2._dep.empty() ) return FFDep::nonlinear( S1 );
  return FFDep::combine( S1, S2, false );
}

inline FFDep
min
( const FFDep&S1, const FFDep&S2 )
{
  if( S1._dep.empty() ) return S2;
  if( S2._dep.empty() ) return S1;
  return FFDep::combine( S1, S2, false );
}

inline FFDep
max
( const FFDep&S1, const FFDep&S2 )
{
  if( S1._dep.empty() ) return S2;
  if( S2._dep.empty() ) return S1;
  return FFDep::combine( S1, S2, false );
}

inline FFDep
inter
( const FFDep&S1, const FFDep&S2 )
{
  if( S1._dep.empty() ) return S2;
  if( S2._dep.empty() ) return S1;
  return FFDep::combine( S1, S2, false );
}

inline FFDep
min
( const unsigned int n, const FFDep*S )
{
  if( n==2 ) return min( S[0], S[1] );
  return min( S[0], min( n-1, S+1 ) );
}

inline FFDep
max
( const unsigned int n, const FFDep*S )
{
  if( n==2 ) return max( S[0], S[1] );
  return max( S[0], max( n-1, S+1 ) );
}

inline FFDep
cos
( const FFDep&S )
{
  return FFDep::nonlinear( S );
}

inline FFDep
sin
( const FFDep &S )
{
  return FFDep::nonlinear( S );
}

inline FFDep
tan
( const FFDep&S )
{
  return FFDep::nonlinear( S );
}

inline FFDep
acos
( const FFDep &S )
{
  return FFDep::nonlinear( S );
}

inline FFDep
asin
( const FFDep &S )
{
  return FFDep::nonlinear( S );
}

inline FFDep
atan
( const FFDep &S )
{
  return FFDep::nonlinear( S );
}

inline bool
operator==
( const FFDep&S1, const FFDep&S2 )
{
  return( S1._dep == S2._dep );
}

inline bool
operator!=
( const FFDep&S1, const FFDep&S2 )
{
  return( S1._dep != S2._dep );
}

inline bool
operator<=
( const FFDep&S1, const FFDep&S2 )
{
  return( S1._dep <= S2._dep );
}

inline bool
operator>=
( const FFDep&S1, const FFDep&S2 )
{
  return( S1._dep >= S2._dep );
}

inline bool
operator<
( const FFDep&S1, const FFDep&S2 )
{
  return( S1._dep < S2._dep );
}

inline bool
operator>
( const FFDep&S1, const FFDep&S2 )
{
  return( S1._dep > S2._dep );
}

} // namespace mc

namespace mc
{

//! @brief Specialization of the structure mc::Op to allow usage of the type mc::Interval for DAG evaluation or as a template parameter in other MC++ classes
template <> struct Op< mc::FFDep >
{
  typedef mc::FFDep FV;
  static FV point( const double c ) { return FV(c); }
  static FV zeroone() { throw typename FFDep::Exceptions( FFDep::Exceptions::UNDEF ); }
  static void I(FV& x, const FV&y) { x = y; }
  static double l(const FV& x) { throw typename FFDep::Exceptions( FFDep::Exceptions::UNDEF ); }
  static double u(const FV& x) { throw typename FFDep::Exceptions( FFDep::Exceptions::UNDEF ); }
  static double abs (const FV& x) { throw typename FFDep::Exceptions( FFDep::Exceptions::UNDEF );  }
  static double mid (const FV& x) { throw typename FFDep::Exceptions( FFDep::Exceptions::UNDEF );  }
  static double diam(const FV& x) { throw typename FFDep::Exceptions( FFDep::Exceptions::UNDEF ); }
  static FV inv (const FV& x) { return mc::inv(x);  }
  static FV sqr (const FV& x) { return mc::sqr(x);  }
  static FV sqrt(const FV& x) { return mc::sqrt(x); }
  static FV log (const FV& x) { return mc::log(x);  }
  static FV xlog(const FV& x) { return x*mc::log(x); }
  static FV fabs(const FV& x) { return mc::fabs(x); }
  static FV exp (const FV& x) { return mc::exp(x);  }
  static FV sin (const FV& x) { return mc::sin(x);  }
  static FV cos (const FV& x) { return mc::cos(x);  }
  static FV tan (const FV& x) { return mc::tan(x);  }
  static FV asin(const FV& x) { return mc::asin(x); }
  static FV acos(const FV& x) { return mc::acos(x); }
  static FV atan(const FV& x) { return mc::atan(x); }
  static FV erf (const FV& x) { return mc::erf(x);  }
  static FV erfc(const FV& x) { return mc::erfc(x); }
  static FV fstep(const FV& x) { return mc::fstep(x); }
  static FV bstep(const FV& x) { return mc::bstep(x); }
  static FV hull(const FV& x, const FV& y) { throw typename FFDep::Exceptions( FFDep::Exceptions::UNDEF ); }
  static FV min (const FV& x, const FV& y) { return mc::min(x,y);  }
  static FV max (const FV& x, const FV& y) { return mc::max(x,y);  }
  static FV arh (const FV& x, const double k) { return mc::exp(-k/x); }
  static FV cheb(const FV& x, const unsigned n) { return mc::cheb(x,n); }
  template <typename X, typename Y> static FV pow(const X& x, const Y& y) { return mc::pow(x,y); }
  static FV monomial (const unsigned int n, const FV* x, const int* k) { throw typename FFDep::Exceptions( FFDep::Exceptions::UNDEF ); }
  static bool inter(FV& xIy, const FV& x, const FV& y) { xIy = mc::inter(x,y); return true; }
  static bool eq(const FV& x, const FV& y) { throw typename FFDep::Exceptions( FFDep::Exceptions::UNDEF ); }
  static bool ne(const FV& x, const FV& y) { throw typename FFDep::Exceptions( FFDep::Exceptions::UNDEF ); }
  static bool lt(const FV& x, const FV& y) { throw typename FFDep::Exceptions( FFDep::Exceptions::UNDEF ); }
  static bool le(const FV& x, const FV& y) { throw typename FFDep::Exceptions( FFDep::Exceptions::UNDEF ); }
  static bool gt(const FV& x, const FV& y) { throw typename FFDep::Exceptions( FFDep::Exceptions::UNDEF ); }
  static bool ge(const FV& x, const FV& y) { throw typename FFDep::Exceptions( FFDep::Exceptions::UNDEF ); }
};

} // namespace mc

#endif
