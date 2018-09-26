// Copyright (C) 2009-2016 Benoit Chachuat, Imperial College London.
// All Rights Reserved.
// This code is published under the Eclipse Public License.

/*!
\page page_ELLIPSOID Ellipsoidal Calculus and Ellipsoidal Arithmetic for Factorable Functions
\author Mario E. Villanueva, Jai Rajyaguru, Boris Houska, Beno&icirc;t Chachuat

An ellipsoid with center \f$c \in \mathbb R^n\f$ and shape matrix \f$Q \in \mathbb S_{+}^n\f$ is defined as 
\f{align*}
  \mathcal E(c,Q) := & \left\{ \left. c + Q^\frac{1}{2} v \ \right| \ \exists v \in \mathbb R^{n}: \ v^T v \leq 1 \right\} \subseteq \mathbb R^{n} .
\f}

As well as constructors and data access/manipulations functions, the class mc::Ellipsoid provides a set of functions for ellipsoidal calculus. Data manipulation functions include:
- checking positive semi-definiteness of the shape matrix (mc::Ellipsoid::psdQ)
- determining the rank of or regularizing the shape matrix (mc::Ellipsoid::rankQ, mc::Ellipsoid::regQ)
- computing the eigenvalues or singular values of the shape matrix (mc::Ellipsoid::eigQ, mc::Ellipsoid::svdQ)
- taking the trace, square root or inverse of the shape matrix (mc::Ellipsoid::trQ, mc::Ellipsoid::sqrtQ, mc::Ellipsoid::invQ)
.
Ellipsoidal calculus includes:
- applying a linear transformation to an ellipsoid (mc::mtimes)
- taking the (exact) intersection of an ellipsoid with a hyperplane (mc::hpintersection)
- computing a (minimum volume) external ellipsoidal approximation of the intersection between an ellipsoid and a halfspace (mc::intersection_ea)
- computing an external ellipsoidal approximation of the geometric (Minkowski) sum of several ellipsoids along a given direction, or a (minimum trace) external ellipsoidal approximation of the geometric (Minkowski) sum of an ellipsoid with an interval box (mc::minksum_ea)
.

Besides ellipsoidal calculus, the classes mc::EllImg and mc::EllVar provide an implementation of ellipsoidal arithmetic in order to enclose the image \f$\mathcal E(c_f,Q_f)\f$ of an \f$n_x\f$-dimensional ellispoid \f$\mathcal E(c_x,Q_x)\f$ under a vector-valued function \f$ f:\mathbb{R}^{n_x}\to\mathbb{R}^{n_f} \f$:
\f{align*}
  \mathcal E(c_f,Q_f) \supseteq & \left\{ f(x) \,\mid\, x\in \mathcal{E}(c_{x},Q_x) \right\}.
\f}
Notice that the exact image \f$\left\{ f(x) \,\mid\, x\in \mathcal{E}(c_{x},Q_x) \right\}\f$ is not an ellipsoid in general.

The class mc::EllImg is derived from mc::Ellipsoid. The implementation of mc::EllImg and mc::EllVar relies on the operator/function overloading mechanism of C++. This makes the computation of the ellipsoidal enclosure for the image of an ellipsoid under a factorable function both simple and intuitive, similar to computing function values in real arithmetic or bounds based on interval, Taylor or Chebyshev model arithmetics (see \ref page_INTERVAL, \ref page_TAYLOR, \ref page_CHEBYSHEV). mc::EllImg stores a column vector CPPL::dcovector and a sparse symmetric matrix CPPL::dssmatrix provided by the LAPACK wrapper <A href="http://cpplapack.sourceforge.net/">CPPLAPACK</A>. The column vector stores the center and the sparse symmetric matrix the shape of a <i>lifted</i> ellipsoid, which is the result of adding extra dimension for each operation participating in the factorable function. Note that the implementation in mc::EllImg and mc::EllVar is <a>not verified</a> in the sense that rounding errors are not accounted for during the propagation.

The classes mc::EllImg and mc::EllVar are templated in the interval type used to bound the nonlinearity of the function, By default, mc::EllImg and mc::EllVar can be used with the non-verified interval type mc::Interval of MC++. For reliability, however, it is recommended to use verified interval arithmetic such as <A href="http://www.ti3.tu-harburg.de/Software/PROFILEnglisch.html">PROFIL</A> (header file <tt>mcprofil.hpp</tt>) or <A href="http://www.math.uni-wuppertal.de/~xsc/software/filib.html">FILIB++</A> (header file <tt>mcfilib.hpp</tt>). 

\section sec_ELLCALC How do I define an ellipsoid and apply ellipsoidal calculus?

In order to define the ellipsoid \f$\mathcal E(c_x,Q_x)\f$ with
\f{align*}
  c_x = & \left(\begin{array}{c} 3\\4\end{array}\right),\ \text{and} & 
  Q_x = & \left(\begin{array}{cc} 5 & 4 \\ 4 & 5 \end{array}\right).
\f}

we proceed as follows:

\code
    const unsigned int n = 2;
    CPPL::dcovector cx(n); CPPL::csymatrix Qx(n);
    cx(0) = 3.;  Qx(0,0) = 5.;
    cx(1) = 4.;  Qx(1,0) = 4.;  Qx(1,1) = 5. ;
    mc::Ellipsoid Ex( Qx, cx ); 
\endcode

This ellipsoid is simply displayed as:

\code
    std::cout << Ex << std::endl;
\endcode

In the present case, the following information is displayed:

\verbatim
center:
 3.00000e+00
 4.00000e+00

shape:
 5.00000e+00 {4.00000e+00}
 4.00000e+00  5.00000e+00 
\endverbatim

In order to illustrate ellipsoidal calculus, suppose that we want to add the interval box \f$[0,0.1]^2\f$ to the foregoing ellispoid and determine an external ellispoidal approximation of this geometric sum. 

\code
   TBC
\endcode

\section sec_ELLIMG How do I compute an ellipsoidal enclosure for the image set of an ellipsoid under a factorable function?

Suppose that we want to compute an ellipsoidal enclosure for the image set of function:
\f[
f(x) = \left(\begin{array}{c}
\log(x_{1})+x^{2}_{2}        \\
\sin(x_{1})-\cos(x_{2})       \\
\end{array} \right) \qquad \text{with} \qquad 
x \in \left\{
\left(\begin{array}{c}
3 \\
4 \\
\end{array} \right)+
\left(\begin{array}{cc}
5 & 4 \\
4 & 5 \\
\end{array} \right)^{1/2}v
:
v^{\rm T}v\leq 1 \right\}.
\f]

For simplicity, the underlying interval bounds are propagated using the default interval type mc::Interval, the required header files 
are:
 
\code
#include "ellimage.hpp"
#include "interval.hpp"

typedef mc::Interval I		;
typedef mc::EllImg<I> EI	;
typedef mc::EllVar<I> EV	;
typedef CPPL::dcovector dcv	;
typedef CPPL::dsymatrix dsm	;
\endcode

First, the host ellipsoidal set for the independent variables \f$x_{1}\f$ and \f$x_{2}\f$ is specified as:
\code 
dcv cx(2)	;     dsm Qx(2)		;
cx(0) = 3.	;     Qx(0,0) = 5.	;
cx(1) = 4.	;     Qx(1,0) = 4.	;	Qx(1,1) = 5.	;
EI Ex( Qx, qx )	;
\endcode

Then, the independent variables themselves are specified as:
\code
EV X1( Ex, 0 ) ;
EV X2( Ex, 1 ) ;
\endcode
If independent interval bounds are known for each variable they can be passed as an optional third argument to the set function.

The dependent variables \f$f_{1}(x)\f$ and \f$f_{2}(c)\f$ are propagated as:
\code
EV F[2] = { log( X[0] ) + mc::sqr( X[1] ),
            sin( X[0] ) - cos( X[1] ) } ;
\endcode

and the lifted ellipsoid can be displayed as:

\code
std::cout << "lifted ellipsoidal image of f =";
Ex.output();
\endcode

\verbatim
lifted ellipsoidal image of f = 
center: 
 3
 4
 18.5
 0.913697
 19.4137
 -0.0590929
 0.012758
 0.0718509
Shape: 
 9.46133 {7.56906}{60.5525}{4.07224}{64.6247}{2.82094}{-4.61268}{-7.43362}
 7.56906  9.46133 {75.6906}{3.25779}{78.9484}{3.52618}{-3.69015}{-7.21632}
 60.5525  75.6906  752.099 {26.0623}{778.161}{28.2094}{-29.5212}{-57.7306}
 4.07224  3.25779  26.0623  2.52547 {28.5878}{1.21416}{-1.98534}{-3.1995}
 64.6247  78.9484  778.161  28.5878  806.749 {29.4236}{-31.5065}{-60.9301}
 2.82094  3.52618  28.2094  1.21416  29.4236  4.40317 {-1.37529}{-5.77847}
 -4.61268  -3.69015  -29.5212  -1.98534  -31.5065  -1.37529  4.34584 {5.72113}
 -7.43362  -7.21632  -57.7306  -3.1995  -60.9301  -5.77847  5.72113  11.4996 
\endverbatim

Finally, the lifted ellipsoid can be projected in the space of dependent variables to obtain the desired image enclosure:
\code
EI Ef = Eflift.get( 2, F );
std::cout << " Ellipsoidal enclosure Ef = " << Ef << std::endl;
\endcode

\verbatim
Ellipsoidal enclosure Ef =
center:
 1.94137e+01
 7.18509e-02
shape:
 8.06749e+02 {-6.09301e+01}
 -6.09301e+01  1.14996e+01 
\endverbatim

After this, the ellipsoid can be manipulated according to the rules of ellipsoidal calculus (see \ref page_ELLIPSOID).
A comparison between the exact image and the ellipsoidal enclosure is presented in the following figure, in orange the exact image and in blue the boundary of the ellipsoidal enclosure.
<CENTER><TABLE BORDER=0>
<TR>
<TD>\image html ELL-2D.png</TD>
</TR>
</TABLE></CENTER>


\section sec_ELL_opt How are the options set for ellipsoidal calculus and ellipsoidal arithmetic?

The class mc::EllImg and mc::Ellipsoid have public members called mc::EllImg::options and mc::Ellipsoid::options (static), respectively, that can be used to set/modify a number of options. Note that mc::EllImg::options is a superset of mc::Ellipsoid::options since mc::EllImg is derived from mc::Ellispoid. For instance, options can be set as follows:
\code
	Ex.options.PREALLOC = 5; 
	Ex.options.CHEBUSE  = false;
\endcode

The full set of available options is reported in the following tables.

<TABLE border="1">
<CAPTION><EM>Options in mc::EllImg::Options: name, type and description</EM></CAPTION>
     <TR><TH><b>Name</b>  <TD><b>Type</b><TD><b>Default</b>
         <TD><b>Description</b>
     <TR><TH><tt>PREALLOC</tt> <TD><tt>unsigned long</tt> <TD>0
         <TD> Number of rows to preallocate in the shape matrix and center vector
     <TR><TH><tt>CHEBUSE</tt> <TD><tt>bool</tt> <TD>false
         <TD> Whether to use Chebyshev expansion to compute a linear approximation and bound the nonlinear dependencies of univariate terms
     <TR><TH><tt>CHEBORDER</tt> <TD><tt>unsigned int</tt> <TD>5
         <TD>Order of the Chebyshev expansion (only of <tt>CHEBUSE = true</tt>)
</TABLE>

<TABLE border="1">
<CAPTION><EM>Options in mc::Ellipsoid::Options: name, type and description</EM></CAPTION>
     <TR><TH><b>Name</b>  <TD><b>Type</b><TD><b>Default</b>
         <TD><b>Description</b>
     <TR><TH><tt>PSDCHK</tt> <TD><tt>bool</tt> <TD>false
         <TD>Whether or not to check positive semi-definiteness of shape matrices
     <TR><TH><tt>PSDTOL</tt> <TD><tt>double</tt> <TD>1e2*MACHPREC
         <TD>Absolute tolerance for positive semi-definiteness check of shape matrices
     <TR><TH><tt>RKTOLA</tt> <TD><tt>double</tt> <TD>MACHPREC
         <TD>Absolute tolerance for rank and regularization of shape matrices
     <TR><TH><tt>RKTOLR</tt> <TD><tt>double</tt> <TD>MACHPREC*1e6
         <TD>Relative tolerance for rank and regularization of shape matrices
     <TR><TH><tt>ROOTTOL</tt> <TD><tt>double</tt> <TD>1e-10
         <TD>Absolute stopping tolerance for root-finding method (objective function value less than ROOTTOL)
     <TR><TH><tt>ROOTSECANT</tt> <TD><tt>bool</tt> <TD>false
         <TD>Whether to use the secant method for root finding
     <TR><TH><tt>ROOTMAXIT</tt> <TD><tt>bool</tt> <TD>0
         <TD>Maximum number of iteration for root-finding method (no maximum when ROOTMAXIT=0)
</TABLE>


\section sec_ELL_err What are the errors encountered during ellipsoidal calculus and ellipsoidal arithmetic?

Errors are managed based on the exception handling mechanism of the C++ language. Each time an error is encountered, a class object of type mc::EllImg::Exceptions or mc::Ellipsoid::Exceptions is thrown, which contains the type of error. It is the user's responsibility to test whether an exception was thrown during the computation of the lifted ellipsoid, and then make the appropriate changes. Should an exception be thrown and not caught by the calling program, the execution will abort.

Possible errors encountered during application of ellipsoidal calculus and ellispoidal arithmetic are reported in the following tables.

<TABLE border="1">
<CAPTION><EM>Errors during the Computation of an Ellipsoidal Image</EM></CAPTION>
     <TR><TH><b>Number</b> <TD><b>Description</b>
     <TR><TH> <tt> 1 </tt>  <TD> Division by zero
     <TR><TH> <tt> 2 </tt>  <TD> Inverse operation with zero in domain
     <TR><TH> <tt> 3 </tt>  <TD> Log operation with non-positive numbers in domain
     <TR><TH> <tt> 4 </tt>  <TD> Square-root operation with negative numbers in domain
     <TR><TH> <tt> 5 </tt>  <TD> Tangent operation with zero in domain of cosine, tan(x) = sin(x)/cos(x)
     <TR><TH> <tt> 6 </tt>  <TD> Sine/Cosine inverse operation with domain outside [-1,1]
     <TR><TH> <tt>-1 </tt>  <TD> Failed to construct ellipsoidal variable
     <TR><TH> <tt>-3 </tt>  <TD> Operation between variables mc::EllVar linked to different images mc::EllImg
     <TR><TH> <tt>-33</tt>  <TD> Feature not yet implemented in mc::EllImg
</TABLE>

<TABLE border="1">
<CAPTION><EM>Errors with mc::Ellipsoid</EM></CAPTION>
     <TR><TH><b>Number</b> <TD><b>Description</b>
     <TR><TH> <tt> 1 </tt> <TD> Non positive-semi definite shape matrix
     <TR><TH> <tt> 2 </tt> <TD> Failure in a LAPACK linear algebra routine
     <TR><TH> <tt> 3 </tt> <TD> Failure in a root-finding routine
</TABLE>

Moreover, exceptions may be thrown by the template parameter class itself.

\section sec_ELL_refs References

Kurzhanskiy, A., A.. and P. Varaiya, <A href="http://code.google.com/p/ellipsoids">"Ellipsoidal Toolbox"</A>, Technical Report UCB/EECS-2006-46, EECS Department, University of California, Berkeley, May 2006.

*/

#ifndef MC__ELLIMAGE_H
#define MC__ELLIMAGE_H

#include <assert.h>
#include <exception>

#include "mcfunc.hpp"
#include "mcop.hpp"
#include "mclapack.hpp"
#include "ellipsoid.hpp"
#include "cmodel.hpp"

#include <fstream>
#include <iomanip>

#undef  MC__ELLIMAGE_DEBUG

namespace mc
{

template< class T > class EllVar;


//! @brief C++ class for ellipsoidal arithmetic - Ellipsoidal image environment
////////////////////////////////////////////////////////////////////////
//! mc::EllImg is a C++ class for definition of ellipsoidal image
//! environment, derived from mc::Ellipsoid. Computation of ellipsoidal
//! image for factorable functions is via the C++ class mc::EllVar. The
//! template parameter corresponds to the type used to propagate
//! variable range. Round-off errors are not accounted for in the
//! computations (non-verified implementation).
////////////////////////////////////////////////////////////////////////
template< class T >
class EllImg: public Ellipsoid
////////////////////////////////////////////////////////////////////////
{

  friend class EllVar<T>;
  
  template< class U > friend  EllVar<U> operator+( const EllVar<U>& );
  template< class U > friend  EllVar<U> operator+( const EllVar<U>&, const EllVar<U>& );
  template< class U > friend  EllVar<U> operator+( const EllVar<U>&, const double );
  template< class U > friend  EllVar<U> operator+( const double, const EllVar<U>& );
  template< class U > friend  EllVar<U> operator+( const EllVar<U>&, const U& );
  template< class U > friend  EllVar<U> operator+( const U&, const EllVar<U>& );
  template< class U > friend  EllVar<U> operator-( const EllVar<U>& );
  template< class U > friend  EllVar<U> operator-( const EllVar<U>&, const EllVar<U>& );
  template< class U > friend  EllVar<U> operator-( const EllVar<U>&, const double );
  template< class U > friend  EllVar<U> operator-( const double, const EllVar<U>& );
  template< class U > friend  EllVar<U> operator-( const EllVar<U>&, const U& );
  template< class U > friend  EllVar<U> operator-( const U&, const EllVar<U>& );
  template< class U > friend  EllVar<U> operator*( const EllVar<U>&, const EllVar<U>& );
  template< class U > friend  EllVar<U> operator*( const EllVar<U>&, const double );			
  template< class U > friend  EllVar<U> operator*( const double, const EllVar<U>& );
  template< class U > friend  EllVar<U> operator*( const EllVar<U>&, const U& );
  template< class U > friend  EllVar<U> operator*( const U&, const EllVar<U>& );
  template< class U > friend  EllVar<U> operator/( const EllVar<U>&, const EllVar<U>& );
  template< class U > friend  EllVar<U> operator/( const EllVar<U>&, const double );			
  template< class U > friend  EllVar<U> operator/( const double, const EllVar<U>& );
  template< class U > friend  EllVar<U> operator/( const EllVar<U>&, const U& );
  template< class U > friend  EllVar<U> operator/( const U&, const EllVar<U>& );

  template< class U > friend  EllVar<U> inv ( const EllVar<U>& );
  template< class U > friend  EllVar<U> exp ( const EllVar<U>& );
  template< class U > friend  EllVar<U> log ( const EllVar<U>& );
  template< class U > friend  EllVar<U> sqrt( const EllVar<U>& );
  template< class U > friend  EllVar<U> sqr ( const EllVar<U>& );
  template< class U > friend  EllVar<U> pow ( const EllVar<U>&, const int );  
  template< class U > friend  EllVar<U> cos ( const EllVar<U>& );
  template< class U > friend  EllVar<U> sin ( const EllVar<U>& );
  template< class U > friend  EllVar<U> tan ( const EllVar<U>& );
  template< class U > friend  EllVar<U> acos( const EllVar<U>& );
  template< class U > friend  EllVar<U> asin( const EllVar<U>& );
  template< class U > friend  EllVar<U> atan( const EllVar<U>& );
  
  public:
    /** @ingroup ELLIPSOID
     *  @{
     */
    //! @brief Exceptions of mc::EllImg
    class Exceptions{
      public:
        //! @brief Enumeration type for EllImg exception handling
        enum TYPE{
          DIV   = 1,	//!< Division by zero scalar
          INV,		//!< Inverse operation with zero in domain
          LOG,		//!< Log operation with non-positive numbers in domain
          SQRT,		//!< Square-root operation with negative numbers in domain
          TAN,		//!< Tangent operation with zero in domain of cosine, tan(x) = sin(x)/cos(x)
          ACOS,		//!< Sine/Cosine inverse operation with domain outside [-1,1]
          INIT  = -1,	//!< Failed to construct ellipsoidal variable EllVar
          EIMG  = -3,	//!< Operation between ellipsoidal variables EllVar linked to different ellipsoidal images EllImg
          UNDEF = -33  	//!< Feature not yet implemented in mc::EllImg
        };
        //! @brief Constructor for error <a>ierr</a>
        Exceptions( TYPE ierr ) : _ierr( ierr ){}
        //! @brief Error flag
        int ierr(){ return _ierr; }
        //! @brief Error description
        std::string what(){
          switch( _ierr ){
            case DIV   : return "mc::EllImg\t Division by zero scalar";
            case INV   : return "mc::EllImg\t Inverse operation with zero in domain";
            case LOG   : return "mc::EllImg\t Log operation with non-positive numbers in domain";
            case SQRT  : return "mc::EllImg\t Square-root operation with negative numbers in domain";
            case TAN   : return "mc::EllImg\t Tangent operation with zero in domain of cosine";
            case ACOS  : return "mc::EllImg\t Inverse sine/cosine operation with domain outside [-1,1]";
            case EIMG  : return "mc::EllImg\t EllVars belong to different ellipsoids, operation not allowed";
            case UNDEF : return "mc::EllImg\t Feature not yet implemented in mc::EllImg class";
            default    : return "mc::EllImg\t Undocumented error";
          }		
        }

      private:
        //! @brief Type of error 
        TYPE _ierr;
    };

    //! @brief Structure containing the options for EllImg
    struct Options
    {
      //! @brief Constructor of mc::EllImg<>::Options
      Options():
        PREALLOC( 0 ), CHEBUSE( false ), CHEBORDER( 5 ), TOL( 1e-10 )
	{}
      //! @brief Sets number of rows to preallocate in the shape matrix and center vector (Default: 0)
      long PREALLOC;
      //! @brief Whether to use Chebyshev models to obtain the linearisation of univariate functions (Default: false)
      bool CHEBUSE;
      //! @brief Order of the Chebyshev model used for the linearisation of univariate functions -- Used only if CHEBUSE is true (Default: 5)
      unsigned CHEBORDER;
      //! @brief Tolerance in minkowski sum (Default: machprec())
      double TOL;
    } options;
    /** @} */

  private:
    //! @brief Shape Matrix of the lifted Ellipsoid
    CPPL::dssmatrix _Q;
    //! @brief Centre of the lifted Ellipsoid
    CPPL::dcovector _q;
    //! @brief Dimension of the dependent variables
    long _nx;
    //! @brief Map between pointers to EllVars (key) and row number
    long _current_row;
    //! @brief pointer to internal Chebyshev model;
    CModel<T>* _CMpt;
    //! @brief variable that is true if a dependency map is available;
    bool _depmap_flag;
    //! @brief dependecy map
    CPPL::dssmatrix _depmap;
    //! @brief index of row where product starts;
    long _ip;
    
#ifdef MC__ELLIMAGE_DEBUG
    //! @brief Output for debugging
    std::ofstream _dbugout;
#endif
	
  public:
    /** @ingroup ELLIPSOID
     *  @{
     */
    //! @brief Returns the shape matrix of the lifted Ellipsoid
    CPPL::dssmatrix Q_lift()
      {	return _Q; }
 
   //! @brief Returns the centre of the lifted Ellipsoid
    CPPL::dcovector c_lift()
      { return _q; }
  
    //! @brief Default constructor
    EllImg();

    //! @brief Constructor for ellipsoid with shape matrix \f$Q\f$ and center \f$c\f$
    EllImg
      ( const CPPL::dsymatrix& Q, const CPPL::dcovector& c=CPPL::dcovector(),
        const CPPL::dssmatrix& depmap = CPPL::dssmatrix() );

    //! @brief Constructor for ellipsoid of dimension \f$n\f$ with shape matrix \f$Q\f$ (lower triangular part stored contiguously and columnwise) and center \f$c\f$
    EllImg
      ( const unsigned int n, const double*Q, const double*c=0,
        const CPPL::dssmatrix& depmap = CPPL::dssmatrix() );

    //! @brief Constructor for ellipsoid enclosing interval vector of radius \f$r\f$ centered at \f$c\f$
    EllImg
      ( const CPPL::dcovector& r, const CPPL::dcovector& c=CPPL::dcovector(),
        const CPPL::dssmatrix& depmap = CPPL::dssmatrix() );

    //! @brief Copy constructor
    EllImg
      ( const EllImg<T>& E );

    //! @brief Destructor
    virtual ~EllImg();
    
    //! @brief Set an ellipsoid identical to <a>E</a>
    EllImg<T>& set
      ( const EllImg<T>& E )
      { Ellipsoid::set( E.Q(), E.c() ); return _reset( E._depmap ); }
    
    //! @brief Set an ellipsoid with shape matrix \f$Q\f$ and center \f$c\f$
    EllImg<T>& set
      ( const CPPL::dsymatrix& Q, const CPPL::dcovector& c=CPPL::dcovector(),
        const CPPL::dssmatrix& depmap = CPPL::dssmatrix() )
      { Ellipsoid::set( Q, c ); return _reset( depmap ); }
    
    //! @brief Set an ellipsoid of dimension \f$n\f$ with shape matrix \f$Q\f$ (lower triangular part stored contiguously and columnwise) and center \f$c\f$
    EllImg<T>& set
      ( const unsigned int n, const double*Q, const double*c=0,
        const CPPL::dssmatrix& depmap = CPPL::dssmatrix() )
      { Ellipsoid::set( n, Q, c ); return _reset( depmap ); }
    
    //! @brief Set an ellipsoidal enclosing interval vector of radius \f$r\f$ centered at \f$c\f$
    EllImg<T>& set
      ( const CPPL::dcovector& r, const CPPL::dcovector& c=CPPL::dcovector(),
        const CPPL::dssmatrix& depmap = CPPL::dssmatrix() )
      { Ellipsoid::set( r, c ); return _reset( depmap ); }

    //! @brief Reset ellipsoidal image to underlying defining ellipsoid
    EllImg<T>& reset()
      { return _reset(); }

    //! @brief Get projection of lifted ellipsoid on variables <a>var</a> 
    EllImg<T> get( unsigned nvar, EllVar<T>* var )
      { return _get( nvar, var ); }

    //! @brief Output lifted ellipsoid to <a>os</a> 
    std::ostream& output( std::ostream&os = std::cout );
    /** @} */

  private:
    //! @brief Computes the Minkowski sum of the ellipsoid and an interval
    void _minksum( const EllVar<T>&, double ); 
	
    //! @brief Computes the Minkowski sum of the ellipsoid and an interval
    void _minksum( long, double ); 
    
    //! @brief Computes the Trace of the Shape matrix for the lifted ellipsoidal image
    double _trQ();

    //! @brief Sets the dependency map for a lifted ellipsoid
    EllImg<T>& _set( const CPPL::dssmatrix& depmap );
		
    //! @brief Resets a lifted ellipsoid
    EllImg<T>& _reset( const CPPL::dssmatrix& depmap = CPPL::dssmatrix() )
      { delete _CMpt; return _set( depmap ); }

    //! @brief projects a given EllVar<>
    EllImg<T> _get( unsigned, EllVar<T>* );
	
    //! @brief univariate update
    void _univupdate( long, long, double, double, double );
	
    //! @brief univariate update used in product
    void _univupdate( long, long, CPPL::drovector, double, double, double );
	
    //! @brief Prototype for univariate function for CModel evaluation
    typedef CVar<T> (univariate_function) ( const CVar<T>&  ) ;
	
    //! @brief typedef for triplet of linearisation parameters (constant, linear, radius of remainder )
    typedef std::tuple<double, double, double> lin_param;

    //! @brief linearise unvariates using Chebyshev models
    lin_param _cmodel_linear( univariate_function, const T& );

    //! @brief linearise power functions using Chebyshev models
    lin_param _cmodel_linear( const int, const T& );

    //! @brief linearise square function
    lin_param _linearize_sqr( const T& );    
};

//! @brief C++ class for ellipsoidal arithmetic - Ellipsoidal image propagation
////////////////////////////////////////////////////////////////////////
//! mc::EllVar is a C++ class for propagation of ellipsoidal image
//! through a factorable function. The template parameter corresponds
//! to the type used to propagate variable range. Round-off errors are
//! not accounted for in the computations (non-verified implementation).
////////////////////////////////////////////////////////////////////////
template< class T >
class EllVar
////////////////////////////////////////////////////////////////////////
{
  friend class EllImg<T>;
  
  template< class U > friend  EllVar<U> operator+( const EllVar<U>& );
  template< class U > friend  EllVar<U> operator+( const EllVar<U>&, const EllVar<U>& );
  template< class U > friend  EllVar<U> operator+( const EllVar<U>&, const double );
  template< class U > friend  EllVar<U> operator+( const double, const EllVar<U>& );
  template< class U > friend  EllVar<U> operator+( const EllVar<U>&, const U& );
  template< class U > friend  EllVar<U> operator+( const U&, const EllVar<U>& );
  template< class U > friend  EllVar<U> operator-( const EllVar<U>& );
  template< class U > friend  EllVar<U> operator-( const EllVar<U>&, const EllVar<U>& );
  template< class U > friend  EllVar<U> operator-( const EllVar<U>&, const double );
  template< class U > friend  EllVar<U> operator-( const double, const EllVar<U>& );
  template< class U > friend  EllVar<U> operator-( const EllVar<U>&, const U& );
  template< class U > friend  EllVar<U> operator-( const U&, const EllVar<U>& );
  template< class U > friend  EllVar<U> operator*( const EllVar<U>&, const EllVar<U>& );
  template< class U > friend  EllVar<U> operator*( const EllVar<U>&, const double );			
  template< class U > friend  EllVar<U> operator*( const double, const EllVar<U>& );
  template< class U > friend  EllVar<U> operator*( const EllVar<U>&, const U& );
  template< class U > friend  EllVar<U> operator*( const U&, const EllVar<U>& );
  template< class U > friend  EllVar<U> operator/( const EllVar<U>&, const EllVar<U>& );
  template< class U > friend  EllVar<U> operator/( const EllVar<U>&, const double );			
  template< class U > friend  EllVar<U> operator/( const double, const EllVar<U>& );
  template< class U > friend  EllVar<U> operator/( const EllVar<U>&, const U& );
  template< class U > friend  EllVar<U> operator/( const U&, const EllVar<U>& );
    
  template< class U > friend  EllVar<U> inv ( const EllVar<U>& );
  template< class U > friend  EllVar<U> exp ( const EllVar<U>& );
  template< class U > friend  EllVar<U> log ( const EllVar<U>& );
  template< class U > friend  EllVar<U> sqrt( const EllVar<U>& );
  template< class U > friend  EllVar<U> sqr ( const EllVar<U>& );
  template< class U > friend  EllVar<U> pow ( const EllVar<U>&, const int );  
  template< class U > friend  EllVar<U> cos ( const EllVar<U>& );
  template< class U > friend  EllVar<U> sin ( const EllVar<U>& );
  template< class U > friend  EllVar<U> tan ( const EllVar<U>& );
  template< class U > friend  EllVar<U> acos( const EllVar<U>& );
  template< class U > friend  EllVar<U> asin( const EllVar<U>& );
  template< class U > friend  EllVar<U> atan( const EllVar<U>& );
							  
  private:
    //! @brief pointer to underlying lifted ellipsoid
    EllImg<T>* _EI;
    //! @brief row index in ellipsoid
    long _RowInd;
    //! @brief flag for constant/interval
    bool _is_constant;
    //! @brief constant value if _is_constant
    double _const_val; 
    //! @brief range of the variable
    T _Range;
		
  public:
    /** @ingroup ELLIPSOID
     *  @{
     */
    //! @brief Default constructor 
    EllVar();
    //! @brief Copy constructor
    EllVar( const EllVar<T>& );
    //! @brief Constructor for constants
    EllVar( const double d );
    //! @brief Constructor for intervals
    EllVar( const T& );
    //! @brief Constructor for intervals
    EllVar( const double l, const double u );
    //! @brief Constructor for variable in ellipsoidal image
    EllVar( EllImg<T>&, const unsigned ); 
    //! @brief Constructor for variable in ellipsoidal image with tailored range
    EllVar( EllImg<T>&, const unsigned, const T& );

    //! @brief Destructor
    virtual ~EllVar(){};
    
    //! @brief set variable the ellipsoidal image environment
    EllVar<T>& set( EllImg<T>& EI, const unsigned i )
    { return _set( EI, i ); } 
    //! @brief set variable in ellipsoidal image environment and tailored range
    EllVar<T>& set( EllImg<T>& EI, const unsigned i, const T& Irange )
    { return _set( EI, i, Irange ); } 
    
    //! @brief get variable range
    T range() const
    { return !_is_constant? _Range: _const_val; }
    //! @brief get pointer to ellipsoidal image
    EllImg<T>* image() const
      { return _EI; }
    //! @brief get pointer to row index
    long index() const
      { return _RowInd; }
  /** @} */

  private:
    //! @brief constructor for internal variables
    EllVar( EllImg<T>*, long ); 
    //! @brief constructor for internal variables used in the product
    EllVar( EllImg<T>*, long, long );
    
    //! @brief set variable in the lifted ellipsoid
    EllVar<T>& _set( EllImg<T>&, const unsigned );    
    //! @brief set variable in the lifted ellipsoid
    EllVar<T>& _set( EllImg<T>&, const unsigned, const T& ); 

    //! @brief get range of EllVar
    const T& _range() const
      { return _Range; }
    //! @brief get/set range of EllVar
    T& _range()
      { return _Range; }		

  public:
    // Public overloads
    EllVar<T>& operator= ( const EllVar<T>& );
    EllVar<T>& operator= ( const double );
    EllVar<T>& operator= ( const T& );

    EllVar<T>& operator+=( const EllVar<T>& );
    EllVar<T>& operator-=( const EllVar<T>& );
    EllVar<T>& operator*=( const EllVar<T>& );
    EllVar<T>& operator/=( const EllVar<T>& );
 
};
 
///////////////////////////////// EllImg ///////////////////////////////////

template <class T> 
inline 
EllImg<T>::EllImg()
: _nx(0), _current_row(0), _CMpt(0), _depmap_flag(false), _depmap(), _ip(-1)
{
#ifdef MC__ELLIMAGE_DEBUG
  _dbugout.open( "debug.log" , std::ios_base::out );
#endif     
}

template <class T> 
inline 
EllImg<T>::EllImg
( const CPPL::dsymatrix& Q, const CPPL::dcovector& c, const CPPL::dssmatrix& depmap )
: Ellipsoid( Q, c )
{
#ifdef MC__ELLIMAGE_DEBUG
  _dbugout.open( "debug.log" , std::ios_base::out );
#endif
  _set( depmap );
}

template <class T> 
inline 
EllImg<T>::EllImg
( const EllImg<T>& E )
: Ellipsoid( E ), options( E.options )
{
#ifdef MC__ELLIMAGE_DEBUG
  _dbugout.open( "debug.log" , std::ios_base::out );
#endif   
  _set( E._depmap );
  //_depmap = E._depmap;
}

template <class T> 
inline 
EllImg<T>::EllImg
( const unsigned int n, const double*Q, const double*c, const CPPL::dssmatrix& depmap )
: Ellipsoid( n, Q, c )
{
#ifdef MC__ELLIMAGE_DEBUG
  _dbugout.open( "debug.log" , std::ios_base::out );
#endif     
  _set( depmap );
}

template <class T> 
inline 
EllImg<T>::EllImg
( const CPPL::dcovector& r, const CPPL::dcovector& c, const CPPL::dssmatrix& depmap )
: Ellipsoid( r, c )
{
#ifdef MC__ELLIMAGE_DEBUG
  _dbugout.open( "debug.log" , std::ios_base::out );
#endif     
  _set( depmap );
}

template <class T> 
inline 
EllImg<T>::~EllImg()
{
#ifdef MC__ELLIMAGE_DEBUG
  _dbugout.close();
#endif     
  delete _CMpt;
}

template <class T> 
inline 
EllImg<T>&
EllImg<T>::_set
( const CPPL::dssmatrix& depmap )
{
  _nx          = Q().n  ;  // resets the number of dependent variables to the dimension of the shape matrix provided
  _current_row = _nx    ;  // resets _current_row to the number of dependent variables 
  _ip          = -1     ;  // resets the row index for the product to the 
  _CMpt        = 0      ;

  // No depmap available
  if( !depmap.n ){
    // set Ellipsoid to  E0(Q0,q0)
    _Q = Q().to_dssmatrix();
    _q = c();
    _depmap_flag = false;
    _depmap.clear();
  }

  //depmap available
  else{ 
    _Q.resize( depmap.n ); 
    _q.resize( depmap.n );
    _q.zero();
    _depmap_flag = true;
    _depmap = depmap;
    // Set the underlying lifted ellipsoid
    for( long j=0; j<Q().n; ++j ){
      // Initialise the varindex-th row of _Q with the varindex-th row of the shape matrix of Ell ; 
      for (long i=0; i<=j; ++i ) _Q.put( i, j, Q(i,j) );
      // Initialise the varindex-th row of _q with the varindex-th row of the centre vector of Ell ;
      _q(j) = c(j);
    }
  }

  return *this;
}

template <class T> 
inline 
EllImg<T>
EllImg<T>::_get
( unsigned nvar, EllVar<T>* var )
{
  CPPL::dsymatrix Q0( nvar ); Q0.zero();
  CPPL::dcovector q0( nvar ); q0.zero();
  for( long j=0 ; j<nvar; ++j  )
  {
  	long prev = var[j]._RowInd; 
  	for ( long i=j ; i<nvar ; ++i )
  	{
   		Q0( i , j ) = _Q( var[i]._RowInd , prev  ) ; 
  		q0( j )     = _q( var[j]._RowInd ) ;
  	} 
  }
  return EllImg<T>( Q0, q0 );
}

template <class T> 
inline 
std::ostream& 
EllImg<T>::output
( std::ostream&os )
{
  if( !_Q.n ) return os;
  const int iprec = 5;
  os << std::scientific << std::setprecision(iprec);
  os << "\nlifted center:\n" << _q;
  os << "lifted shape matrix:\n" << _Q;
  return os;
}

template <class T> 
inline 
void
EllImg<T>::_univupdate
( long i      ,
  long k      , 
  double c0   ,
  double c1   ,
  double eta    )
{
////  #ifdef MC__ELLIMAGE_DEBUG
////  _dbugout << std::scientific << std::setprecision(3) << std::right;
////  _dbugout << "univariate update starts: i= " << i <<" k= "<< k <<std::endl;
////  _dbugout << "q \n" ;
////  _dbugout << _q <<std::endl;
////  _dbugout << "Q \n" ;
////  _dbugout << _Q <<std::endl;
////  #endif
  // Update centre
  _q( i ) = c0;
  // Update matrix when the shape matrix has been initialised with a dependency map
  if( _depmap_flag )
  {
 	  // Update shape matrix
	  for(long j = 0; j < i ; ++j) if( _depmap.isListed( i, j ) ) _Q(i, j) = c1 * _Q(k,j) ;
	  // Update diagonal element
	  _Q( i, i ) = std::pow(c1,2) * _Q(k,k) ; 
  }
  else // No dependecy map available
  {
	  // Update shape matrix
	  for(long j = 0; j < i ; ++j) if( _Q.isListed( k, j ) ) _Q.put(i, j, c1 * _Q(k,j) );
	  // Update diagonal element
	  _Q.put( i, i, std::pow(c1,2) * _Q(k,k) );
  }
  // Minkowski sum with an interval centered at 0 with radius eta
  _minksum( i , eta );
////  #ifdef MC__ELLIMAGE_DEBUG
////  _dbugout << std::scientific << std::setprecision(3) << std::right;
////  _dbugout << "univariate update ends: i= " << i <<" k= "<< k <<std::endl;
////  _dbugout << "q \n" ;
////  _dbugout << _q <<std::endl;
////  _dbugout << "Q \n" ;
////  _dbugout << _Q <<std::endl;
////  #endif
}

template <class T> 
inline 
void
EllImg<T>::_univupdate
( long i             ,
  long k             , 
  CPPL::drovector rQ , 
  double c0          ,
  double c1          ,
  double eta           )
{
  // Update centre
  _q( i ) = c0;
  // Update matrix when the shape matrix has been initialised with a dependency map
  if( _depmap_flag )
  {
 	  // Update shape matrix
	  for(long j = 0; j < i ; ++j) if( _depmap.isListed( i, j ) ) _Q(i, j) = c1 * _Q(k,j) ;
	  // Update diagonal element
	  if( _depmap.isListed( i, i ) ) _Q( i, i ) = std::pow(c1,2) * _Q(k,k) ; 
  }
  else // No dependecy map available
  {
	  // Update shape matrix
	  for(long j = 0; j < i ; ++j) _Q.put(i, j, c1 * _Q(k,j) );
	  // Update diagonal element
	  _Q.put( i, i, std::pow(c1,2) * _Q(k,k) );
  }
  // Minkowski sum with an interval centered at 0 with radius eta
  _minksum( i , eta );
}

//!@brief linearise using Chebyshev models
template <class T> 
inline 
typename EllImg<T>::lin_param
EllImg<T>::_linearize_sqr
( const T& domain )
{
  if( options.CHEBUSE ) return _cmodel_linear( sqr , domain );

  // Compute linear approximation of sqr(m+r*x) with x \in [-1,1] using minimax linearisation
  double c0, c1, eta;
  double m = Op<T>::mid( domain );
  double r = 0.5 * Op<T>::diam( domain );
  double c_sec=0., x_tan=0., c_tan=0.;
  c_sec = m*m+r*r;
  c1   = 2.0*m*r;
  x_tan = (c1-2.*m*r)/(2.*r*r);
  c_tan = sqr(m+r*x_tan) - c1*x_tan;
  eta   = 0.5*(c_sec - c_tan);
  c0    = c_sec - eta;
  c1    =  c1 / r;
  return std::make_tuple( c0, c1, eta );
}

//!@brief linearise using Chebyshev models
template <class T> 
inline
typename EllImg<T>::lin_param
EllImg<T>::_cmodel_linear
( univariate_function f, const T& domain )
{	
  // (re)initialise Chebyshev model if it hasnt been used
  // or is of different order
  if( !_CMpt ) _CMpt = new CModel<T>( 1, options.CHEBORDER );
  else if( _CMpt->nord() != options.CHEBORDER )
    { delete _CMpt; _CMpt = new CModel<T>( 1, options.CHEBORDER ); }

  CVar<T> CVX( _CMpt, 0 , domain );
  CVar<T> CVF = f( CVX );
  double c0 = CVF.C().constant();
  double c1 = CVF.linear( 0 , true );
  double eta = 0.5 * Op<T>::diam(CVF.B());
  return std::make_tuple( c0, c1, eta );
} 

template <class T> 
inline
typename EllImg<T>::lin_param
EllImg<T>::_cmodel_linear
( const int n, const T& domain )
{	
  // (re)initialise Chebyshev model if it hasnt been used
  // or is of different order
  if( !_CMpt ) _CMpt = new CModel<T>( 1, options.CHEBORDER );
  else if( _CMpt->nord() != options.CHEBORDER )
    { delete _CMpt; _CMpt = new CModel<T>( 1, options.CHEBORDER ); }

  CVar<T> CVX( _CMpt, 0 , domain );
  CVar<T> CVF = pow( CVX, n);
  double c0 = CVF.C().constant();
  double c1 = CVF.linear( 0 , true );
  double eta = 0.5 * Op<T>::diam(CVF.B());
  return std::make_tuple( c0, c1, eta );
}

template <class T>
inline
void 
EllImg<T>::_minksum
( const EllVar<T>& EllVar1, double rad )
{
  long i = EllVar1._RowInd;
  _minksum( i, rad );	
}

template <class T>
inline
void 
EllImg<T>::_minksum
( const long i, const double rad )
{
  #ifdef MC__ELLIMAGE_DEBUG
  _dbugout << std::scientific << std::setprecision(3) << std::right;
  _dbugout << "Minkowski sum starts: i= " << i <<std::endl;
  _dbugout << "q \n" ;
  _dbugout << _q <<std::endl;
  _dbugout << "Q \n" ;
  _dbugout << _Q <<std::endl;
  #endif
  double TOL = options.TOL, EPS = machprec(), strQ = 0.0; 
  
  for( long j=0; j<=i ; ++j ){
    if( _Q.isListed(j,j) ) strQ += _Q(j,j)/(_Q(j,j)+TOL); // for some reason this loops modifies the diagonal elements of the product block if the isListed is not used ... 
  }
  strQ = std::sqrt(strQ);
  double sqrR  =  rad/std::sqrt(_Q(i,i)+TOL);
  double kappa = strQ + sqrR + EPS;
  _Q          *= (kappa/(strQ+EPS));
  _Q(i,i)     += (std::pow(rad,2)*kappa)/(sqrR+EPS);

////  #ifdef MC__ELLIMAGE_DEBUG
////  _dbugout << std::scientific << std::setprecision(3) << std::right;
////  _dbugout << "Minkowski sum ends: i= " << i <<std::endl;
////  _dbugout << "q \n" ;
////  _dbugout << _q <<std::endl;
////  _dbugout << "Q \n" ;
////  _dbugout << _Q <<std::endl;
////  #endif
}  

template <class T> 
inline
double 
EllImg<T>::_trQ
()
{
  if( !_Q.n ) return 0.;
  double tr(_Q(0,0));
  for( unsigned int i=1; i<_Q.n; i++ ) tr += _Q(i,i);
  return tr;
}

/////////////////////////////////  EllVar  ///////////////////////////////////

template <class T> 
inline 
EllVar<T>::EllVar
( ) :
  _EI         ( NULL )  ,
  _RowInd     ( -1 )    ,
  _is_constant( false ) ,
  _const_val  ( 0 )		,
  _Range      ( T(0) ) 
{}

template <class T> 
inline 
EllVar<T>::EllVar
( const EllVar<T>& EllVar1 ) :
  _EI         ( EllVar1._EI )          ,  
  _RowInd     ( EllVar1._RowInd )      ,
  _is_constant( EllVar1._is_constant ) ,
  _const_val  ( EllVar1._const_val )   ,
  _Range      ( EllVar1._Range )
{ }  

template <class T> 
inline 
EllVar<T>::EllVar
( const double d ) :
  _EI         ( NULL )   , // a "double" EllVar lives outside the EllImg 
  _RowInd     ( -1 )     , // it is also a constant and has a "trivial"
  _is_constant( true )   , // range. Test to identify it
  _const_val  ( d )      , // if( _EI == NULL && _is_constant  ) .
  _Range      ( T(d) )      // Range should not be accessed.
{ }  

template <class T> 
inline 
EllVar<T>::EllVar
( const T& B ) :
  _EI         ( NULL )  ,  // an "interval" EllVar lives outside the Ellimg
  _RowInd     ( -1 )    ,  // it is NOT a constant, has a trivial constant
  _is_constant( false ) ,  // constant value and a nontrivial range. Test
  _const_val  ( 0. )    ,  // to identify it is
  _Range      ( B )        // if( _EI == NULL && !_is_constant ) .
{ }

template <class T> 
inline 
EllVar<T>::EllVar
( const double l, const double u ) :
  _EI         ( NULL )  ,  // an "interval" EllVar lives outside the Ellimg
  _RowInd     ( -1 )    ,  // it is NOT a constant, has a trivial constant
  _is_constant( false ) ,  // constant value and a nontrivial range. Test
  _const_val  ( 0. )    ,  // to identify it is
  _Range      ( l, u )     // if( _EI == NULL && !_is_constant ) .
{ }

template <class T> 
inline  
EllVar<T>::EllVar
( EllImg<T>& EI, const unsigned i )
{
  _set( EI, i );
}

template <class T> 
inline  
EllVar<T>::EllVar
( EllImg<T>& EI, const unsigned i, const T& Irange )
{
  _set( EI, i, Irange );
}

template <class T> 
inline  
EllVar<T>::EllVar
( EllImg<T>* EI , 
  long RowInd       ) :
  _EI         ( EI )     ,
  _RowInd     ( RowInd ) ,
  _is_constant( false )  ,
  _const_val  ( 0.  )    ,
  _Range      ( T() )    
{ 
  if( !_EI ) throw typename EllImg<T>::Exceptions( EllImg<T>::Exceptions::INIT );
  // The ellipsoid is lifted if it has not been initialised with a dependecy map
  if( ! EI->_depmap_flag ) 
  {
	  // Checks if preallocation is needed
	  if( EI->options.PREALLOC && RowInd >= EI->_Q.n   )
	  {
	  	long dim = EI->options.PREALLOC;
	  	EI->_Q.stretch( dim );
	  	EI->_q.stretch( dim );
	  }
	  else if( !EI->options.PREALLOC && RowInd >= EI->_Q.n )
	  {
	  	EI->_Q.stretch( 1 );
	  	EI->_q.stretch( 1 );
	  }
  }
  EI->_current_row++;
}

template <class T> 
inline  
EllVar<T>::EllVar
( EllImg<T>* EI   , 
  long RowInd     ,
  long current_row  ) :
  _EI         ( EI )     ,
  _RowInd     ( RowInd ) ,
  _is_constant( false )  ,
  _const_val  ( 0.  )    ,
  _Range      ( T() )    
{ 
  if( !_EI ) throw typename EllImg<T>::Exceptions( EllImg<T>::Exceptions::INIT );
  // The ellipsoid is lifted if it has not been initialised with a dependecy map
  if( ! EI->_depmap_flag ) 
  {
	  // Checks if preallocation is needed
	  if( EI->options.PREALLOC && RowInd >= EI->_Q.n   )
	  {
	  	long dim = EI->options.PREALLOC;
	  	EI->_Q.stretch( dim );
	  	EI->_q.stretch( dim );
	  }
	  else if( !EI->options.PREALLOC && RowInd >= EI->_Q.n )
	  {
	  	EI->_Q.stretch( 1 );
	  	EI->_q.stretch( 1 );
	  }
  }
  EI->_current_row = current_row;
}

template <class T> 
inline 
EllVar<T>&
EllVar<T>::_set
( EllImg<T>& EI, const unsigned i )
{
  if( i >= EI._nx )
    throw typename EllImg<T>::Exceptions( EllImg<T>::Exceptions::INIT ); 
  _EI          = &EI;
  _RowInd      = i;
  _is_constant = false;
  _const_val   = 0.;
  _Range = EI._q(i) + std::sqrt(EI._Q( i,i )) * T(-1.,1.);
  return *this;
}

template <class T> 
inline 
EllVar<T>&
EllVar<T>::_set
( EllImg<T>& EI, const unsigned i, const T& Irange )
{
  if( i >= EI._nx )
    throw typename EllImg<T>::Exceptions( EllImg<T>::Exceptions::INIT ); 
  _EI          = &EI;
  _RowInd      = i;
  _is_constant = false;
  _const_val   = 0.;
  if( !Op<T>::inter( _Range, EI._q(i) + std::sqrt(EI._Q( i,i )) * T(-1.,1.), Irange ) )
    throw typename EllImg<T>::Exceptions( EllImg<T>::Exceptions::INIT ); 
  return *this;
}

template <class T> 
inline 
EllVar<T>&
EllVar<T>::operator=
( const EllVar<T>& EllVar1 ) 
{
  _EI          = EllVar1._EI          ;
  _RowInd      = EllVar1._RowInd      ;
  _is_constant = EllVar1._is_constant ;
  _const_val   = EllVar1._const_val   ;
  _Range       = EllVar1._Range       ;
  return *this; 
}

template <class T> 
inline 
EllVar<T>&
EllVar<T>::operator=
( const double d ) 
{
  _EI          = NULL ;
  _RowInd      = -1   ;
  _is_constant = true ;
  _const_val   = d    ;
  _Range       = T(d)  ;
  return *this; 
}

template <class T> 
inline 
EllVar<T>&
EllVar<T>::operator=
( const T& B ) 
{
  _EI          = NULL  ;
  _RowInd      = -1    ;
  _is_constant = false ;
  _const_val   = 0     ;
  _Range       = B     ;
  return *this; 
}

template <typename T> inline EllVar<T>&
EllVar<T>::operator +=
( const EllVar<T>&E1 )
{
   EllVar<T> E2( *this );
   *this = E2 + E1;
   return *this;
}

template <typename T> inline EllVar<T>&
EllVar<T>::operator -=
( const EllVar<T>&E1 )
{
   EllVar<T> E2( *this );
   *this = E2 - E1;
   return *this;
}

template <typename T> inline EllVar<T>&
EllVar<T>::operator *=
( const EllVar<T>&E1 )
{
   EllVar<T> E2( *this );
   *this = E2 * E1;
   return *this;
}

template <typename T> inline EllVar<T>&
EllVar<T>::operator /=
( const EllVar<T>&E1 )
{
   EllVar<T> E2( *this );
   *this = E2 / E1;
   return *this;
}

/////////////////////////////////  Operators  ///////////////////////////////////

template <class T> 
inline 
EllVar<T> 
operator+
( const EllVar<T>& EllVar1 )
{
  return EllVar1;
}

template <class T> 
inline 
EllVar<T>  
operator+
( const EllVar<T>& EllVar1 , 
  const EllVar<T>& EllVar2  )
{
  // The left operand is not an ellipsoidal variables
  if( !EllVar1._EI ){
    if( EllVar1._is_constant ) return EllVar2 + EllVar1._const_val;
    return EllVar2 + EllVar1._Range;
  }
  // The right operand is not an ellipsoidal variables
  if( !EllVar2._EI ){
    if( EllVar2._is_constant ) return EllVar1 + EllVar2._const_val;
    return EllVar1 + EllVar2._Range;
  }
  // The operands correspond to different ellipsoids
  if( EllVar1._EI != EllVar2._EI )
    throw typename EllImg<T>::Exceptions( EllImg<T>::Exceptions::EIMG );

  EllImg<T>* EI = EllVar1._EI;
  long i          = EI->_current_row;
  long k          = EllVar1._RowInd;
  long l          = EllVar2._RowInd;
  // Construct Evar3 to return (increases _current_row)
  EllVar<T> EllVar3( EI, i );
#ifdef MC__ELLIMAGE_DEBUG
  EI->_dbugout << std::scientific << std::setprecision(3) << std::right;
  EI->_dbugout << "+ starts: i= " << i <<" k= "<< k <<" l= "<< l <<std::endl;
  EI->_dbugout << "q \n" ;
  EI->_dbugout << EI->_q <<std::endl;
  EI->_dbugout << "Q \n" ;
  EI->_dbugout << EI->_Q <<std::endl;
#endif
  // Update Centre
  EI->_q( i )  =  EI->_q(k) + EI->_q(l);

  // Update matrix when the shape matrix has been initialised with a dependency map
  if( EI-> _depmap_flag ){
    // Update current row in the Shape matrix
    for ( long j = 0; j < i ; ++j )
      if( EI->_depmap.isListed(i,j) ) EI->_Q.put( i,j, EI->_Q(k,j) + EI->_Q(l,j)  );
    // Update Diagonal element
    if( EI->_depmap.isListed(i,i) ) EI->_Q.put( i,i, EI->_Q(k,k) + 2.*(EI->_Q(l,k)) + EI->_Q(l,l) );
  }

  // No dependency map available 
  else{
    // Update current row in the Shape matrix
    for ( long j = 0; j < i ; ++j )
      if( EI->_Q.isListed(k,j) || EI->_Q.isListed(l,j) ) EI->_Q.put( i,j, EI->_Q(k,j) + EI->_Q(l,j)  );
    // Update Diagonal element
    EI->_Q.put( i,i, EI->_Q(k,k) + 2.*(EI->_Q(l,k)) + EI->_Q(l,l) );
  }

  // Save range of Variable 
  T Range = EI->_q(i) + std::sqrt(EI->_Q(i,i))*T(-1.,1.); 
  EllVar3._range() =  Range;
#ifdef MC__ELLIMAGE_DEBUG
  EI->_dbugout << std::scientific << std::setprecision(3) << std::right;
  EI->_dbugout << "+ ends: i= " << i <<" k= "<< k <<" l= "<< l <<std::endl;
  EI->_dbugout << "q \n" ;
  EI->_dbugout << EI->_q <<std::endl;
  EI->_dbugout << "Q \n" ;
  EI->_dbugout << EI->_Q <<std::endl;
#endif

  return EllVar3;
}

template <class T> 
inline 
EllVar<T>  
operator+
( const EllVar<T>& EllVar1 , 
  const double  DVal )
{
  // Neither of the operands are ellipsoidal variables
  if( !EllVar1._EI ){
    if( EllVar1._is_constant ) return DVal + EllVar1._const_val;
    return DVal + EllVar1._Range;
  }

  EllImg<T>* EI = EllVar1._EI;
  long i          = EI->_current_row;
  long k          = EllVar1._RowInd;
  // Construct Evar3 to return (increases _current_row)
  EllVar<T> EllVar3( EI, i );
  // Update Centre
  EI->_q( i )  =  EI->_q(k) + DVal;
  // Update matrix when the shape matrix has been initialised with a dependency map
  if( EI-> _depmap_flag )
  {
  	  // Update current row in the Shape matrix
	  for ( long j = 0; j < i ; ++j )
	  {
  		if( EI->_depmap.isListed(i,j) ) EI->_Q.put(i,j, EI->_Q(k,j) );
  	  }
	  // Update Diagonal element
	  if( EI->_depmap.isListed(i,i) ) EI->_Q.put( i,i, EI->_Q(k,k) );
  }
  else // No dependency map available 
  {
	  // Update current row in the Shape matrix
	  for ( long j = 0; j < i ; ++j ) 
	  {
			if( EI->_Q.isListed(k,j) ) EI->_Q.put(i,j, EI->_Q(k,j) );
	  }
	  // Update Diagonal element
	  EI->_Q.put( i,i, EI->_Q(k,k) );
  }
  // Save range of Variable 
  T Range          = EI->_q(i) + std::sqrt(EI->_Q(i,i))*T(-1.,1.); 
  EllVar3._range() =  Range;
	
  return EllVar3;
 }

template <class T> 
inline 
EllVar<T>  
operator+
( const double  DVal, 
  const EllVar<T>& EllVar1 )
{
  return EllVar1 + DVal;
}

template <class T> 
inline 
EllVar<T>  
operator+
( const EllVar<T>& EllVar1 , 
  const T&  	   IVal      )
{
  // Neither of the operands are ellipsoidal variables
  if( !EllVar1._EI ){
    if( EllVar1._is_constant ) return IVal + EllVar1._const_val;
    return IVal + EllVar1._Range;
  }

  EllImg<T>* EI = EllVar1._EI;
  long i          = EI->_current_row;
  long k          = EllVar1._RowInd;
  // Construct Evar3 to return (increases _current_row)
  EllVar<T> EllVar3( EI, i );
  // Update Centre
  EI->_q( i )  =  EI->_q(k) + Op<T>::mid(IVal) ; // Offset by the midpoint of the interval
  // Update matrix when the shape matrix has been initialised with a dependency map
  if( EI-> _depmap_flag )
  {
	EI->_minksum( i , 0.5*Op<T>::diam(IVal) );
  }
  else // No dependency map available 
  {
	EI->_minksum( i , 0.5*Op<T>::diam(IVal) );
  }
  // Save range of Variable 
  T Range          = EI->_q(i) + std::sqrt(EI->_Q(i,i))*T(-1.,1.); 
  EllVar3._range() = Range;
	
  return EllVar3;
}

template <class T> 
inline 
EllVar<T>  
operator+
( const T&  	   IVal    , 
  const EllVar<T>& EllVar1   )
{
  return EllVar1 + IVal ;
}
 
template <class T> 
inline 
EllVar<T>  
operator-
( const EllVar<T>& EllVar1 )
{
  // The operand is not an ellipsoidal variable
  if( !EllVar1._EI ){
    if( EllVar1._is_constant ) return -EllVar1._const_val;
    return -EllVar1._Range;
  }

  EllImg<T>* EI = EllVar1._EI;
  long i          = EI->_current_row;
  long k          = EllVar1._RowInd;
  // Construct Evar3 to return (increases _current_row)
  EllVar<T> EllVar3( EI, i );
  // Update Centre
  EI->_q( i )  =  - EI->_q(k) ;
  // Update matrix when the shape matrix has been initialised with a dependency map
  if( EI-> _depmap_flag )
  {
  	  // Update current row in the Shape matrix
	  for ( long j = 0; j < i ; ++j )
	  {
  		if( EI->_depmap.isListed(i,j) ) EI->_Q.put( i,j, - EI->_Q(k,j) );
  	  }
	  // Update Diagonal element
	 if( EI->_depmap.isListed(i,i) ) EI->_Q.put( i,i, EI->_Q(k,k) );
  }
  else // No dependency map available 
  {
	  // Update current row in the Shape matrix
	  for ( long j = 0; j < i ; ++j ) 
	  {
	  	if( EI->_Q.isListed(k,j) ) EI->_Q.put( i,j, - EI->_Q(k,j) );
	  }
	  // Update Diagonal element
	  EI->_Q.put( i,i, EI->_Q(k,k) );
  }
  // Save range of Variable 
  T Range          = EI->_q(i) + std::sqrt(EI->_Q(i,i))*T(-1.,1.); 
  EllVar3._range() =  Range;
  
  return EllVar3;
}

template <class T> 
inline 
EllVar<T>  
operator-
( const EllVar<T>& EllVar1 , 
  const EllVar<T>& EllVar2  )
{
  if( EllVar1._EI == NULL && EllVar1._is_constant &&
      EllVar2._EI == NULL && EllVar2._is_constant    ) // Both EllVar1 and EllVar2 are doubles
  { 
  	return EllVar1._const_val - EllVar2._const_val;
  }
  else if( EllVar1._EI == NULL && EllVar1._is_constant ) //  EllVar1 is a double
  {
     return EllVar1._const_val - EllVar2 ;
  }
  else if( EllVar2._EI == NULL && EllVar2._is_constant  ) // EllVar2 is a double
  {
  	return EllVar1 - EllVar2._const_val ;
  }
  else if( EllVar1._EI == NULL && ! EllVar1._is_constant && 
  		   EllVar2._EI == NULL && ! EllVar2._is_constant    ) // Both EllVar1 and EllVar2 are intervals
  {
  	return EllVar1._Range - EllVar2._Range; // Computes interval and calls constructor EllVar<T>( const T& )
  }
  else if( EllVar1._EI == NULL && ! EllVar1._is_constant ) // EllVar1 is an interval
  {
  	return EllVar1._Range - EllVar2 ;
  }
  else if( EllVar2._EI == NULL && ! EllVar2._is_constant ) // EllVar2 is an interval
  {
  	return EllVar1 + (-1.*EllVar2._Range);
  }
  else if( EllVar1._EI != EllVar2._EI ) // EllVar1 and EllVar2 are in different ellipsoids
  {	
   	throw typename EllImg<T>::Exceptions( EllImg<T>::Exceptions::EIMG );
  }
  else   //EllVar1 and EllVar2 are 'common' EllVar<T>s
  {
	  EllImg<T>* EI = EllVar1._EI;
	  long i          = EI->_current_row;
	  long k          = EllVar1._RowInd;
	  long l          = EllVar2._RowInd;
	  // Construct Evar3 to return (increases _current_row)
	  EllVar<T> EllVar3( EI, i );
	  // Update Centre
	  EI->_q( i )  =  EI->_q(k) - EI->_q(l);
	  // Update matrix when the shape matrix has been initialised with a dependency map
	  if( EI-> _depmap_flag )
	  {
	  	  // Update current row in the Shape matrix
		  for ( long j = 0; j < i ; ++j )
		  {
	  		if( EI->_depmap.isListed(i,j) ) EI->_Q.put(i,j, EI->_Q(k,j) - EI->_Q(l,j)  );
	  	  }
		  // Update Diagonal element
		 if( EI->_depmap.isListed(i,i) )  EI->_Q.put( i,i, EI->_Q(k,k) - 2.*(EI->_Q(l,k)) + EI->_Q(l,l) );
	  }
	  else // No dependency map available 
	  {
		  // Update current row in the Shape matrix
		  for ( long j = 0; j < i ; ++j ) 
		  {
			if( EI->_Q.isListed(k,j) || EI->_Q.isListed(l,j) ) EI->_Q.put(i,j, EI->_Q(k,j) - EI->_Q(l,j)  );
		  }
		  // Update Diagonal element
		  EI->_Q.put( i,i, EI->_Q(k,k) - 2.*(EI->_Q(l,k)) + EI->_Q(l,l) );
	  }
	  // Save range of Variable 
	  T Range          = EI->_q(i) + std::sqrt(EI->_Q(i,i))*T(-1.,1.); 
	  EllVar3._range() =  Range;

	  return EllVar3;
  }
}

template <class T> 
inline 
EllVar<T>  
operator-
( const EllVar<T>& EllVar1 , 
  const double  DVal )
{
  // Neither of the operands are ellipsoidal variables
  if( !EllVar1._EI ){
    if( EllVar1._is_constant ) return EllVar1._const_val - DVal;
    return EllVar1._Range - DVal;
  }

  EllImg<T>* EI = EllVar1._EI;
  long i          = EI->_current_row;
  long k          = EllVar1._RowInd;
  // Construct Evar3 to return (increases _current_row)
  EllVar<T> EllVar3( EI, i );
  // Update Centre
  EI->_q( i )  =  EI->_q(k) - DVal;
  // Update matrix when the shape matrix has been initialised with a dependency map
  if( EI-> _depmap_flag )
  {
  	  // Update current row in the Shape matrix
	  for ( long j = 0; j < i ; ++j )
	  {
  		if( EI->_depmap.isListed(i,j) ) EI->_Q.put(i,j, EI->_Q(k,j) );
  	  }
	  // Update Diagonal element
	  if( EI->_depmap.isListed(i,i) ) EI->_Q.put( i,i, EI->_Q(k,k) );
  }
  else // No dependency map available 
  {
	  // Update current row in the Shape matrix
	  for ( long j = 0; j < i ; ++j )  
	  {
		if( EI->_Q.isListed(k,j) ) EI->_Q.put(i,j, EI->_Q(k,j) );
	  }
	  // Update Diagonal element
	  EI->_Q.put( i,i, EI->_Q(k,k) );
  }
  // Save range of Variable 
  T Range          = EI->_q(i) + std::sqrt(EI->_Q(i,i))*T(-1.,1.); 
  EllVar3._range() =  Range;

  return EllVar3;
}

template <class T> 
inline 
EllVar<T> 
operator-
( const double  DVal, 
  const EllVar<T>& EllVar1 )
{
  // Neither of the operands are ellipsoidal variables
  if( !EllVar1._EI ){
    if( EllVar1._is_constant ) return DVal - EllVar1._const_val;
    return DVal - EllVar1._Range;
  }

  EllImg<T>* EI = EllVar1._EI;
  long i       = EI->_current_row;
  long k       = EllVar1._RowInd;
  // Construct Evar3 to return (increases _current_row)
  EllVar<T> EllVar3( EI, i );
  // Update Centre
  EI->_q( i )  =  DVal - EI->_q(k) ;
  // Update matrix when the shape matrix has been initialised with a dependency map
  if( EI-> _depmap_flag )
  {
  	  // Update current row in the Shape matrix
	  for ( long j = 0; j < i ; ++j )
	  {
  		if( EI->_depmap.isListed(i,j) ) EI->_Q.put(i,j, - EI->_Q(k,j) );
  	  }
	  // Update Diagonal element
	  if( EI->_depmap.isListed(i,i) ) EI->_Q.put( i,i, EI->_Q(k,k) );
  }
  else // No dependency map available 
  {
	  // Update current row in the Shape matrix
	  for ( long j = 0; j < i ; ++j ) 
	  {
		if( EI->_Q.isListed(k,j) ) EI->_Q.put(i,j, - EI->_Q(k,j) );
	  }
	  // Update Diagonal element
	  EI->_Q.put( i,i, EI->_Q(k,k) );
  }
  // Save range of Variable 
  T Range          = EI->_q(i) + std::sqrt(EI->_Q(i,i))*T(-1.,1.); 
  EllVar3._range() =  Range;

  return EllVar3;
}

template <class T> 
inline 
EllVar<T>  
operator-
( const EllVar<T>& EllVar1 , 
  const T& IVal              )
{
  // Neither of the operands are ellipsoidal variables
  if( !EllVar1._EI ){
    if( EllVar1._is_constant ) return EllVar1._const_val - IVal;
    return EllVar1._Range - IVal;
  }

  EllImg<T>* EI   = EllVar1._EI;
  long i          = EI->_current_row;
  long k          = EllVar1._RowInd;
  
  // Construct Evar3 to return (increases _current_row)
  EllVar<T> EllVar3( EI, i );
  // Update Centre
  EI->_q( i )  =  EI->_q(k) + Op<T>::mid( -1.*IVal ) ; // shift by the center of the interval  // Update matrix when the shape matrixhas been initialised with a dependency map
  if( EI-> _depmap_flag )
  {
  	EI->_minksum( i , 0.5*Op<T>::diam( -1.*IVal ) ) ;	
  }
  else // No dependency map available 
  {
	EI->_minksum( i , 0.5*Op<T>::diam( -1.*IVal ) ) ;
  }
  // Save range of Variable 
  T Range          = EI->_q(i) + std::sqrt(EI->_Q(i,i))*T(-1.,1.); 
  EllVar3._range() =  Range;

  return EllVar3;
}

template <class T> 
inline 
EllVar<T>  
operator-
( const T&  IVal, 
  const EllVar<T>& EllVar1 )
{
  // Neither of the operands are ellipsoidal variables
  if( !EllVar1._EI ){
    if( EllVar1._is_constant ) return IVal - EllVar1._const_val;
    return IVal - EllVar1._Range;
  }

  EllImg<T>* EI   = EllVar1._EI;
  long i          = EI->_current_row;
  long k          = EllVar1._RowInd;
  
  // Construct Evar3 to return (increases _current_row)
  EllVar<T> EllVar3( EI, i );
  // Update Centre
  EI->_q( i )  =  Op<T>::mid( IVal ) + EI->_q(k) ; // shift by the center of the interval  
  // Update matrix when the shape matrixhas been initialised with a dependency map
  if( EI-> _depmap_flag )
  {
  	EI->_minksum( i , 0.5*Op<T>::diam( IVal ) ) ;	
  }
  else // No dependency map available 
  {
	EI->_minksum( i , 0.5*Op<T>::diam( IVal ) ) ;
  }
  // Save range of Variable 
  T Range          = EI->_q(i) + std::sqrt(EI->_Q(i,i))*T(-1.,1.); 
  EllVar3._range() =  Range;

  return EllVar3;
}
template <class T> 
inline 
EllVar<T>  
operator*
( const EllVar<T>& EllVar0 , 
  const EllVar<T>& EllVar1  )
{
  return (sqr(EllVar0+EllVar1)-sqr(EllVar0-EllVar1))/4.;
}
/*
template <class T> 
inline 
EllVar<T>  
operator*
( const EllVar<T>& EllVar0 , 
  const EllVar<T>& EllVar1  )
{
  if( EllVar0._EI == NULL && EllVar0._is_constant &&
      EllVar1._EI == NULL && EllVar1._is_constant    ) // Both EllVar0 and EllVar1 are doubles
  { 
  	return EllVar0._const_val * EllVar1._const_val;
  }
  else if( EllVar0._EI == NULL && EllVar0._is_constant ) //  EllVar0 is a double
  {
     return EllVar0._const_val * EllVar1 ;
  }
  else if( EllVar1._EI == NULL && EllVar1._is_constant  ) // EllVar1 is a double
  {
  	return EllVar0 * EllVar1._const_val ;
  }
  else if( EllVar0._EI == NULL && ! EllVar0._is_constant && 
  		   EllVar1._EI == NULL && ! EllVar1._is_constant    ) // Both EllVar0 and EllVar1 are intervals
  {
  	return EllVar0._Range * EllVar1._Range; // Computes interval and calls constructor EllVar<T>( const T& )
  }
  else if( EllVar0._EI == NULL && ! EllVar0._is_constant ) // EllVar0 is an interval
  {
  	throw typename EllImg<T>::Exceptions( EllImg<T>::Exceptions::UNDEF);
  }
  else if( EllVar1._EI == NULL && ! EllVar1._is_constant ) // EllVar1 is an interval
  {
  	throw typename EllImg<T>::Exceptions( EllImg<T>::Exceptions::UNDEF );
  }
  else if( EllVar0._EI != EllVar1._EI ) // EllVar1 and EllVar2 are in different ellipsoids
  {	
   	throw typename EllImg<T>::Exceptions( EllImg<T>::Exceptions::EIMG );
  }
  else   //EllVar1 and EllVar2 are 'common' EllVar<T>s
  {
	  EllImg<T>* EI = EllVar0._EI;
	  long i;
	  long k  = EllVar0._RowInd;
	  long l  = EllVar1._RowInd;
	  T domain_EllVar0  = EI->_q( k ) + std::sqrt( EI->_Q(k,k) ) *T(-1.,1);
	  T domain_EllVar1  = EI->_q( l ) + std::sqrt( EI->_Q(l,l) ) *T(-1.,1);
	  if( EI->_ip == -1 )
	  {	
	  	EI->_ip = EI->_current_row    ;	// The product block starts at the current_row 
		if( ! EI->_depmap_flag && EI->_ip  + 7 > EI->_Q.n   ) // If there are not at least 7 more rows  
		{													  // to accomodate the product block 			
			EI->_Q.stretch( 7 )     ;   // Stretch to make space for the product block
		  	EI->_q.stretch( 7 )     ;	// in every row, updates are performed until i (i,i) is the diagonal element
	  		for( long j = EI->_current_row ; j < EI->_current_row + 6 ; ++j ) EI->_q(j) = 0.; 
	  	}
	  	i  = EI->_current_row + 6 ; // Index for product result is the last row created 
	  }
	  else i = EI->_current_row; 
	  long ii = EI->_ip		  ;	// internal row counter for product block
	  long kk = ii            ; // internal counter for argument in univariate operations;
	  long ll				  ; // internal counter for 2nd argument
	  double result; 
	  // Construct EllVar
	  EllVar<T> EllVar3( EI, i , i+1 );
////	  // Update depmap for the product block
////	  if( EI->_depmap_flag )
////	  {
////	  	// update for v8 = v7 - v6
////	  	long ip = i , kp = EI->_ip + 4 , lp = EI->_ip + 5 ; // index for v8, v7 and v6
////		for( long jp = 0 ; jp < i  ; ++jp )  
////		{
////			if( EI->_depmap.isListed( ip,jp ) )
////			{ 
////				EI->_depmap.put( jp,kp,1 );
////				EI->_depmap.put( jp,lp,1 );
////			}
////		}
////		EI->_depmap.put( kp,kp,1 );
////		EI->_depmap.put( kp,lp,1 );
////		EI->_depmap.put( lp,lp,1 );
////		// update for v7 = 1/4 * v5
////		ip = EI->_ip + 5 ; kp = EI->_ip + 3;
////		for( long jp = 0 ; jp < i  ; ++jp )  
////		{
////			if( EI->_depmap.isListed( ip,jp ) )
////			{ 
////				EI->_depmap.put( jp,kp,1 );
////			}
////		}
////		EI->_depmap.put( kp,kp,1 );
////		// update for v6 = 1/4 * v4
////		ip = EI->_ip + 4 ; kp = EI->_ip + 2;
////		for( long jp = 0 ; jp < i  ; ++jp )  
////		{
////			if( EI->_depmap.isListed( ip,jp ) )
////			{ 
////				EI->_depmap.put( jp,kp,1 );
////			}
////		}
////		EI->_depmap.put( kp,kp,1 );
////	    // update for v5 = sqr( v3 )
////		ip = EI->_ip + 3 ; kp = EI->_ip + 1;
////		for( long jp = 0 ; jp < i  ; ++jp )  
////		{
////			if( EI->_depmap.isListed( ip,jp ) )
////			{ 
////				EI->_depmap.put( jp,kp,1 );
////			}
////		}
////		EI->_depmap.put( kp,kp,1 );
////		// update for v5 = sqr( v3 )
////		ip = EI->_ip + 2 ; kp = EI->_ip ;
////		for( long jp = 0 ; jp < i  ; ++jp )  
////		{
////			if( EI->_depmap.isListed( ip,jp ) )
////			{ 
////				EI->_depmap.put( jp,kp,1 );
////			}
////		}
////		EI->_depmap.put( kp,kp,1 );
////	  	std::cout << " DEPMAP B4 : \n";
////	  	std::cout << EI->_depmap << std::endl; 
////	  }
	#ifdef MC__ELLIMAGE_DEBUG
	  EI->_dbugout << std::scientific << std::setprecision(3) << std::right;
	  EI->_dbugout << "Product starts: i= " << i <<" k= "<< k << " l= " << l <<std::endl;
	  EI->_dbugout << "Product block starts: "  << EI->_ip << std::endl;  
	  EI->_dbugout << " _q =\n" ;
	  EI->_dbugout <<  EI->_q << std::endl;
	  EI->_dbugout << " _Q =\n" ;
	  EI->_dbugout <<  EI->_Q << std::endl;
	#endif 
	  // compute v2 = ( v0 + v1 )
//	  if( EI->_depmap_flag )
//	  {
//		  for( long j = 0 ; j <= i ; ++j ) 
//		  {
//		  	if(  EI->_depmap.isListed(ii,j) ) EI->_Q.put( ii,j, EI->_Q( k,j ) + EI->_Q( l,j ) );
//		  }
//		  if( EI->_depmap.isListed(ii,ii) ) EI->_Q.put( ii,ii, EI->_Q( k,k ) + 2.* EI->_Q( l,k ) + EI->_Q( l,l ) );
//	  } 
//	  else
//	  {
	  	for( long j = 0 ; j <= i ; ++j ) 
	  	{
	  		if( EI->_Q.isListed(k,j) || EI->_Q.isListed(l,j)  )
	  		{ 
	  			result = EI->_Q( k,j ) + EI->_Q( l,j );
	  			if( ! isequal(result,0.) ) EI->_Q.put( ii,j, result );
	  		}	
		}
		result = EI->_Q( k,k ) + 2.* EI->_Q( l,k ) + EI->_Q( l,l );	// Diagonal
		if( ! isequal(result,0.) ) EI->_Q.put( ii,ii, result );
//	  } 
	  EI->_q( ii ) = EI->_q( k ) + EI->_q( l ); 
	  ii += 1; // increase row counter for product block
	 #ifdef MC__ELLIMAGE_DEBUG
	  EI->_dbugout << std::scientific << std::setprecision(3) << std::right;
	  EI->_dbugout << " After v2 : \n" ;
	  EI->_dbugout << " _q =\n" ;
	  EI->_dbugout <<  EI->_q << std::endl;
	  EI->_dbugout << " _Q =\n" ;
	  EI->_dbugout <<  EI->_Q << std::endl;
	#endif 
	  // compute v3 = ( v0 - v1 )
//	  if( EI->_depmap_flag )
//	  {
//	  	for( long j = 0 ; j <= i ; ++j ) if(  EI->_depmap.isListed(ii,j) ) EI->_Q.put( ii,j, EI->_Q( k,j ) - EI->_Q( l,j ) );
//	  	if( EI->_depmap.isListed(ii,ii) ) EI->_Q.put( ii,ii, EI->_Q( k,k ) - 2.* EI->_Q( l,k ) + EI->_Q( l,l ) ); 
//	  } 
//	  else
//  	  {
	  	for( long j = 0 ; j <= i ; ++j ) 
	  	{
	  		if( EI->_Q.isListed(k,j) || EI->_Q.isListed(l,j)  )  
	  		{	
	  			double result = EI->_Q( k,j ) - EI->_Q( l,j ) ;
	  			if( ! isequal(result,0.) ) EI->_Q.put( ii,j, result );
  			}
		}
	  	result =  EI->_Q( k,k ) - 2.* EI->_Q( l,k ) + EI->_Q( l,l ); //Diagonal
	  	if( ! isequal(result,0.) ) EI->_Q.put( ii,ii,result ); 
//	  }
	  EI->_q( ii ) = EI->_q( k ) - EI->_q( l ); 
	  T domainv3 =  EI->_q( ii ) + std::sqrt( EI->_Q( ii , ii ) ) * T(-1.,1.); // Compute domain for v5 before Msum (in v4) 
	  ii += 1; // increase row counter for product block
	#ifdef MC__ELLIMAGE_DEBUG
	  EI->_dbugout << std::scientific << std::setprecision(3) << std::right;
	  EI->_dbugout << " After v3 : \n" ;
	  EI->_dbugout << " _q =\n" ;
	  EI->_dbugout <<  EI->_q << std::endl;
	  EI->_dbugout << " _Q =\n" ;
	  EI->_dbugout <<  EI->_Q << std::endl;
	#endif 
      // compute v4 = ( v2 )^2
	  T domain = EI->_q( kk ) + std::sqrt( EI->_Q(kk,kk) )*T(-1.,1.);
	  double c_0, c_1, eta;
	  std::tie( c_0, c_1, eta ) = EI->_linearize_sqr( domain );
//	  if( EI->_depmap_flag )
//	  {
//	  	for( long j = 0 ; j <= i ; ++j ) if(  EI->_depmap.isListed(ii,j) )EI->_Q.put( ii,j, c_1*EI->_Q( kk ,j ) );
//	  	if( EI->_depmap.isListed(ii,ii) )  EI->_Q.put( ii,ii, std::pow(c_1,2)*EI->_Q(kk,kk) );
//	  } 
//	  else
//	  {
	  	for( long j = 0 ; j <= i ; ++j )
	  	{
	  		if( EI->_Q.isListed( kk ,j ) )
	  		{
	  			result = c_1*EI->_Q( kk ,j );
	  			if(! isequal(result,0.) ) EI->_Q.put( ii,j, result );
	  		}
	  	}
	  	result = std::pow(c_1,2)*EI->_Q(kk,kk);
	  	if(! isequal(result,0.) ) EI->_Q.put( ii,ii, result );
//	  }      
	  EI->_q( ii ) = c_0;    
	  EI->_minksum( ii, eta );
	  ii += 1; // increase row counter for product block
	  kk += 1; // increase counter for argument
	#ifdef MC__ELLIMAGE_DEBUG
	  EI->_dbugout << std::scientific << std::setprecision(3) << std::right;
	  EI->_dbugout << " After v4 : \n" ;
	  EI->_dbugout << " _q =\n" ;
	  EI->_dbugout <<  EI->_q << std::endl;
	  EI->_dbugout << " _Q =\n" ;
	  EI->_dbugout <<  EI->_Q << std::endl;
	#endif
	  // compute v5 = ( v3 )^2  
	  std::tie( c_0, c_1, eta ) = EI->_linearize_sqr( domainv3 );                
//	  if( EI->_depmap_flag )
//	  {
//	  	for( long j = 0 ; j <= i ; ++j ) if(  EI->_depmap.isListed(ii,j) ) EI->_Q.put( ii,j, c_1*EI->_Q( kk ,j ) );
//	  	if( EI->_depmap.isListed(ii,ii) )  EI->_Q.put( ii,ii, std::pow(c_1,2)*EI->_Q(kk,kk) );   
//	  } 
//	  else
//  	  {
	  	for( long j = 0 ; j <= i ; ++j )
	  	{
	  		if( EI->_Q.isListed( kk ,j )  )
	  		{
	  			result = c_1*EI->_Q( kk ,j );
	  			if(! isequal(result,0.) ) EI->_Q.put( ii,j, result );
	  		}
	  	} 
	  	result = std::pow(c_1,2)*EI->_Q(kk,kk);
	  	if(! isequal(result,0.) ) EI->_Q.put( ii,ii, result );   
//	  }  
	  EI->_q( ii ) = c_0;
	  EI->_minksum( ii, eta );
	  ii += 1; // increase row counter for product block
	  kk += 1; // increase counter for argument
	#ifdef MC__ELLIMAGE_DEBUG
	  EI->_dbugout << std::scientific << std::setprecision(3) << std::right;
	  EI->_dbugout << " After v5 : \n" ;
	  EI->_dbugout << " _q =\n" ;
	  EI->_dbugout <<  EI->_q << std::endl;
	  EI->_dbugout << " _Q =\n" ;
	  EI->_dbugout <<  EI->_Q << std::endl;
	#endif 
	  // compute v6 = (1/4)*( v4 )
//	  if( EI->_depmap_flag )
//	  {
//	  	for( long j = 0 ; j <= i ; ++j ) if(  EI->_depmap.isListed(ii,j) ) EI->_Q.put( ii,j, (1./4.)*EI->_Q( kk ,j ) ); 
//	  	if( EI->_depmap.isListed(ii,ii) ) EI->_Q.put( ii,ii,(1./16.)*EI->_Q(kk,kk) );
//	  } 
//	  else
//	  {
	  	for( long j = 0 ; j <= i ; ++j )
	  	{
	  		if( EI->_Q.isListed( kk,j ) )
	  		{
	  			result = (1./4.)*EI->_Q( kk ,j ) ;
	  			if( ! isequal(result,0.) ) EI->_Q.put( ii,j, result );
	  		}
	  	}  
	  	result = (1./16.)*EI->_Q(kk,kk); 
	  	if( ! isequal(result,0.) ) EI->_Q.put( ii,ii,result );
//	  }    
	  EI->_q( ii ) = (1./4.)*EI->_q( kk );
	  ii += 1; // increase row counter for product block
	  kk += 1; // increase counter for argument
  	#ifdef MC__ELLIMAGE_DEBUG
	  EI->_dbugout << std::scientific << std::setprecision(3) << std::right;
	  EI->_dbugout << " After v6 : \n" ;
	  EI->_dbugout << " _q =\n" ;
	  EI->_dbugout <<  EI->_q << std::endl;
	  EI->_dbugout << " _Q =\n" ;
	  EI->_dbugout <<  EI->_Q << std::endl;
	#endif 
	  // compute v7 = (1/4)*( v5 )
//	  if( EI->_depmap_flag )
//	  {
//	  	for( long j = 0 ; j <= i ; ++j ) if(  EI->_depmap.isListed(ii,j) ) EI->_Q.put( ii,j, (1./4.)*EI->_Q( kk ,j ) );
//	  	if( EI->_depmap.isListed(ii,ii) )  EI->_Q.put( ii,ii,(1./16.)*EI->_Q(kk,kk) );  
//	  } 
//	  else
//	  {
	  	for( long j = 0 ; j <= i ; ++j )
	  	{
	  		if( EI->_Q.isListed( kk ,j ) )
	  		{
	  			result = (1./4.)*EI->_Q( kk ,j );
	  			if( ! isequal( result, 0.) ) EI->_Q.put( ii,j, result );
	  		}
	  	} 
	  	result = (1./16.)*EI->_Q(kk,kk);
	  	if( ! isequal( result, 0.) ) EI->_Q.put( ii,ii, result );  
//	  } 
	  EI->_q( ii ) = (1./4.)*EI->_q( kk );
	  kk += 1;
	  ll  = kk+1;
	#ifdef MC__ELLIMAGE_DEBUG
	  EI->_dbugout << std::scientific << std::setprecision(3) << std::right;
	  EI->_dbugout << " After v7 : \n" ;
	  EI->_dbugout << " _q =\n" ;
	  EI->_dbugout <<  EI->_q << std::endl;
	  EI->_dbugout << " _Q =\n" ;
	  EI->_dbugout <<  EI->_Q << std::endl;
	#endif 
	  // compute v8 = ( v6 - v7 )
	  if( EI->_depmap_flag )
	  {
	  	for( long j = 0 ; j < i ; ++j ) if(  EI->_depmap.isListed(i,j) ) EI->_Q.put( i,j, EI->_Q( kk,j ) - EI->_Q( ll,j ) );
	  	if( EI->_depmap.isListed(i,i) )  EI->_Q.put( i,i, EI->_Q( kk,kk ) - 2.* EI->_Q( ll,kk ) + EI->_Q( ll,ll ) ); 
	  } 
	  else
	  {
	  	for( long j = 0 ; j < i ; ++j )
	  	{
	  		if( EI->_Q.isListed( kk,j ) || EI->_Q.isListed( ll,j )  )
	  		{
	  			result = EI->_Q( kk,j ) - EI->_Q( ll,j )  ;
	  			if( ! isequal(result,0.) ) EI->_Q.put( i,j, result );
	  		}
	  	}
	  	result = EI->_Q( kk,kk ) - 2.* EI->_Q( ll,kk ) + EI->_Q( ll,ll );
	  	if( ! isequal(result,0.) ) EI->_Q.put( i,i, result ); 
	  }  
	  EI->_q( i ) = EI->_q( kk ) - EI->_q( ll ); 
	#ifdef MC__ELLIMAGE_DEBUG
	  EI->_dbugout << std::scientific << std::setprecision(3) << std::right;
	  EI->_dbugout << " After v8 : \n" ;
	  EI->_dbugout << " _q =\n" ;
	  EI->_dbugout <<  EI->_q << std::endl;
	  EI->_dbugout << " _Q =\n" ;
	  EI->_dbugout <<  EI->_Q << std::endl;
	#endif 
	  //cleanup the product block
	  for (long ipp= EI->_ip ; ipp <= ii ; ++ipp )
	  {
	  	for (long jp= 0 ; jp <= i ; ++jp ) 
	  	{
		  	EI -> _Q.del( ipp , jp ) ;
		  	 
	  	}
	  	EI->_Q.del( ipp,ipp );
	  }
//	  if( EI->_depmap_flag )
//	  {
//		  for (long ipp= EI->_ip ; ipp <= ii ; ++ipp )
//		  {
//		  	for (long jp= 0 ; jp <= i ; ++jp ) 
//		  	{
//			  	EI -> _depmap.del( ipp , jp ) ;
//			  	 
//		  	}
//		  	EI->_depmap.del( ipp,ipp );
//		  }	  
//	  }
//  	  std::cout << " DEPMAP After : \n";
//  	  std::cout << EI->_depmap << std::endl; 

	  //save bound the product
	  T Range                  ;
	  Op<T>::inter( Range, EI->_q(i)+std::sqrt(EI->_Q( i,i ))*T(-1.,1. ), domain_EllVar0 * domain_EllVar1 );
	  EllVar3._range() =  Range; 
#ifdef MC__ELLIMAGE_DEBUG
	  EI->_dbugout << std::scientific << std::setprecision(3) << std::right;
	  EI->_dbugout << "Product ends: i= " << i <<" k= "<< k << " l= " << l <<std::endl;
	  EI->_dbugout << "Product block starts: "  << EI->_ip << std::endl;  
	  EI->_dbugout << " _q =\n" ;
	  EI->_dbugout <<  EI->_q << std::endl;
	  EI->_dbugout << " _Q =\n" ;
	  EI->_dbugout <<  EI->_Q << std::endl;
#endif 
	  
	  return EllVar3;
  }
}
*/
template <class T> 
inline 
EllVar<T> 
operator*
( const EllVar<T>& EllVar1 , 
  const double  DVal )
{
  // Neither of the operands are ellipsoidal variables
  if( !EllVar1._EI ){
    if( EllVar1._is_constant ) return EllVar1._const_val * DVal;
    return EllVar1._Range * DVal;
  }

  EllImg<T>* EI = EllVar1._EI;
  long i        = EI->_current_row;
  long k        = EllVar1._RowInd;
  // Construct Evar3 to return (increases _current_row)
  EllVar<T> EllVar3( EI, i );
  // Update Centre
  EI->_q( i )  =  EI->_q(k) * DVal;
  // Update matrix when the shape matrix has been initialised with a dependency map
  if( EI-> _depmap_flag )
  {
  	  // Update current row in the Shape matrix
	  for ( long j = 0; j < i ; ++j )
	  {
  		if( EI->_depmap.isListed(i,j) ) EI->_Q(i,j) = DVal * EI->_Q(k,j) ;
  	  }
	  // Update Diagonal element
	  EI->_Q(i,i) = DVal*DVal*EI->_Q(k,k) ;
  }
  else // No dependency map available 
  {
	  // Update current row in the Shape matrix
	  for ( long j = 0; j < i ; ++j ) 
	  {
		EI->_Q.put(i, j, DVal*EI->_Q(k,j) );
	  }
	  // Update Diagonal element
	  EI->_Q.put( i, i, DVal*DVal*EI->_Q(k,k) );
  }
  // Save range of Variable 
  T Range          = EI->_q(i) + std::sqrt(EI->_Q(i,i))*T(-1.,1.); 
  EllVar3._range() =  Range;
 
  return EllVar3;
}

template <class T> 
inline 
EllVar<T>  
operator*
( const double  DVal, 
  const EllVar<T>& EllVar1 )
{
  return EllVar1 * DVal;
}

template <class T> 
inline 
EllVar<T> 
operator*
( const EllVar<T>& EllVar1 , 
  const T&  IVal )
{
  // Neither of the operands are ellipsoidal variables
  if( !EllVar1._EI ){
    if( EllVar1._is_constant ) return IVal * EllVar1._const_val;
    return IVal * EllVar1._Range;
  }

  EllImg<T>* EI = EllVar1._EI;
  long i        = EI->_current_row;
  long k        = EllVar1._RowInd;
  throw typename EllImg<T>::Exceptions( EllImg<T>::Exceptions::UNDEF );
//  // Construct Evar3 to return (increases _current_row)
//  EllVar<T> EllVar3( EI, i );
//  // Update Centre
//  EI->_q( i )  =  EI->_q(k) * DVal;
//  // Update matrix when the shape matrix has been initialised with a dependency map
//  if( EI-> _depmap_flag )
//  {

//  }
//  else // No dependency map available 
//  {

//  }
//  // Save range of Variable 
//  T Range          = EI->_q(i) + std::sqrt(EI->_Q(i,i))*T(-1.,1.); 
//  EllVar3._range() =  Range;
// 
//  return EllVar3;
}

template <class T> 
inline 
EllVar<T> 
operator*
( const T&  IVal           , 
  const EllVar<T>& EllVar1   )
{
  // Neither of the operands are ellipsoidal variables
  if( !EllVar1._EI ){
    if( EllVar1._is_constant ) return IVal * EllVar1._const_val;
    return IVal * EllVar1._Range;
  }

  EllImg<T>* EI = EllVar1._EI;
  long i        = EI->_current_row;
  long k        = EllVar1._RowInd;
  throw typename EllImg<T>::Exceptions( EllImg<T>::Exceptions::UNDEF );
//  // Construct Evar3 to return (increases _current_row)
//  EllVar<T> EllVar3( EI, i );
//  // Update Centre
//  EI->_q( i )  =  EI->_q(k) * DVal;
//  // Update matrix when the shape matrix has been initialised with a dependency map
//  if( EI-> _depmap_flag )
//  {

//  }
//  else // No dependency map available 
//  {

//  }
//  // Save range of Variable 
//  T Range          = EI->_q(i) + std::sqrt(EI->_Q(i,i))*T(-1.,1.); 
//  EllVar3._range() =  Range;
// 
//  return EllVar3;
}

template <class T> 
inline 
EllVar<T>  
operator/
( const EllVar<T>& EllVar1, 
  const EllVar<T>& EllVar2 )
{
  if( EllVar1._EI == NULL && EllVar1._is_constant &&
      EllVar2._EI == NULL && EllVar2._is_constant    ) // Both EllVar1 and EllVar2 are doubles
  { 
  	return EllVar1._const_val / EllVar2._const_val;
  }
  else if( EllVar1._EI == NULL && EllVar1._is_constant ) //  EllVar1 is a double
  {
     return EllVar1._const_val * inv(EllVar2) ;
  }
  else if( EllVar2._EI == NULL && EllVar2._is_constant  ) // EllVar2 is a double
  {
  	return (1./EllVar2._const_val) * EllVar1 ;
  }
  else if( EllVar1._EI == NULL && ! EllVar1._is_constant && 
  		   EllVar2._EI == NULL && ! EllVar2._is_constant    ) // Both EllVar1 and EllVar2 are intervals
  {
  	return EllVar1._Range / EllVar2._Range; // Computes interval and calls constructor EllVar<T>( const T& )
  }
  else if( EllVar1._EI == NULL && ! EllVar1._is_constant ) // EllVar1 is an interval
  {
  	throw typename EllImg<T>::Exceptions( EllImg<T>::Exceptions::UNDEF);
  }
  else if( EllVar2._EI == NULL && ! EllVar2._is_constant ) // EllVar2 is an interval
  {
  	throw typename EllImg<T>::Exceptions( EllImg<T>::Exceptions::UNDEF );
  }
  else if( EllVar1._EI != EllVar2._EI ) // EllVar1 and EllVar2 are in different ellipsoids
  {	
   	throw typename EllImg<T>::Exceptions( EllImg<T>::Exceptions::EIMG );
  }
  else   //EllVar1 and EllVar2 are 'common' EllVar<T>s
  {
  	return EllVar1 * inv( EllVar2 );
  }
}

template <class T> 
inline 
EllVar<T> 
operator/
( const EllVar<T>& EllVar1, 
  const double  DVal  )
{
  if( isequal( DVal, 0. ) )
    throw typename EllImg<T>::Exceptions( EllImg<T>::Exceptions::DIV );
  return ( 1./DVal ) * EllVar1;
}

template <class T> 
inline 
EllVar<T>  
operator/
( const double  DVal, 
  const EllVar<T>& EllVar1 )
{
  return DVal * inv( EllVar1 );
}

template <class T> 
inline 
EllVar<T> 
operator/
( const EllVar<T>& EllVar1, 
  const T&  IVal  )
{
  return ( 1./IVal ) * EllVar1;
}

template <class T> 
inline 
EllVar<T>  
operator/
( const T& IVal           , 
  const EllVar<T>& EllVar1  )
{
  return IVal * inv( EllVar1 );
}

template <class T> 
inline 
EllVar<T> 
inv
( const EllVar<T>& EllVar1 )
{
  if( EllVar1._EI == NULL && EllVar1._is_constant ) //  EllVar1 is a double
  {
     return 1./( EllVar1._const_val ) ;
  }
  else if( EllVar1._EI == NULL && ! EllVar1._is_constant ) // EllVar1 is an interval
  {
  	return inv( EllVar1._Range ) ;
  }
  else   //EllVar1 is a 'common' EllVar<T>s
  {
	EllImg<T>* EI = EllVar1._EI;
	long i          = EI->_current_row;
	long k          = EllVar1._RowInd;

	// Construct Evar3 to return (increases _current_row)
	EllVar<T> EllVar3( EI, i );
	// bound the argument EllVar1 in the current ellipsoid
	T domain = EllVar1._range();
	if( Op<T>::l(domain) <= 0. && Op<T>::u(domain) >= 0. )
	{	
		throw typename EllImg<T>::Exceptions( EllImg<T>::Exceptions::INV );
	} 
	// Compute linear approximation of inv( m + r*x ) with x \in [-1,1] using Chebyshev models
	double c_0, c_1, eta;
	if( EI->options.CHEBUSE )
	{
		std::tie( c_0, c_1, eta ) = EI->_cmodel_linear( inv , domain );
	}
	else // Compute linear approximation of inv( m + r*x ) with x \in [-1,1] using minimax linearisation
	{
		double m = Op<T>::mid( domain );
		double r = 0.5 * Op<T>::diam( domain );
		double c_sec=0., x_tan=0., c_tan=0.;
		c_sec   = m/(std::pow(m,2) - std::pow(r,2));
		c_1     = r/(std::pow(r,2) - std::pow(m,2));
		if ( m > 0 ) x_tan =  (std::sqrt(-r/c_1)-m)/r;
		else         x_tan =  1./((std::sqrt(-r/c_1)-m)/r);
		c_tan   = 1./(m+r*x_tan)-c_1*x_tan;
		eta     = (c_sec - c_tan)/2.;
		c_0     = c_sec - eta;
		c_1     = c_1/r;
	} 
	// Update matrix
	EI->_univupdate( i , k , c_0 , c_1 , eta ); 
	// Save range of Variable
	T Range                  ;
	Op<T>::inter( Range, EI->_q(i)+std::sqrt(EI->_Q( i,i ))*T(-1.,1. ), 1./(domain) );
	EllVar3._range() =  Range;

	#ifdef MC__ELLIMAGE_DEBUG
	EI->_dbugout << std::scientific << std::setprecision(3) << std::right;
	EI->_dbugout << "inv ends: i= " << i <<" k= "<< k <<std::endl;
	EI->_dbugout << "Range, q , Q\n" ;
	for( long i = 0 ; i < EI->_Q.n ; ++i )
	{
		EI->_dbugout << "\t" << EI->_q(i) << "\t" << EI->_Q.row(i); 
	}
	EI->_dbugout << std::endl;
	#endif 
	return EllVar3;
  }  
}
 
template <class T> 
inline 
EllVar<T> 
exp
( const EllVar<T>& EllVar1 )
{
  if( EllVar1._EI == NULL && EllVar1._is_constant ) //  EllVar1 is a double
  {
     return std::exp( EllVar1._const_val ) ;
  }
  else if( EllVar1._EI == NULL && ! EllVar1._is_constant ) // EllVar1 is an interval
  {
  	return exp( EllVar1._Range ) ;
  }
  else   //EllVar1 is a 'common' EllVar<T>s
  {
  	EllImg<T>* EI = EllVar1._EI;
  	long i          = EI->_current_row;
  	long k          = EllVar1._RowInd;
	// Construct Evar3 to return (increases _current_row)
	EllVar<T> EllVar3( EI, i );
	// bound the argument EllVar1 in the current ellipsoid
	T domain = EllVar1._range();
	// Compute linear approximation of exp( m + r*x ) with x \in [-1,1] using Chebyshev models
	double c_0, c_1, eta;
	if( EI->options.CHEBUSE )
	{
		std::tie( c_0, c_1, eta ) = EI->_cmodel_linear( exp , domain );
	}
	else // Compute linear approximation of exp( m + r*x ) with x \in [-1,1] using minimax linearisation
	{
		double m = Op<T>::mid( domain );
		double r = 0.5 * Op<T>::diam( domain );
		double c_sec=0., x_tan=0., c_tan=0.;
		c_sec  = 0.5*(std::exp(m-r)+std::exp(m+r));
		c_1    = 0.5*(std::exp(m+r)-std::exp(m-r));
		x_tan  = (std::log(c_1/r) - m)/r;
		c_tan  = std::exp(m+r*x_tan)-c_1*x_tan;
		eta = (c_sec - c_tan)/2.0;
		c_0 = c_sec - eta;
		c_1 = c_1/r;
	}
	// Update matrix 
	EI->_univupdate( i , k , c_0 , c_1 , eta ); 
	// Save range of Variable
	T Range                  ;
	Op<T>::inter( Range, EI->_q(i)+std::sqrt(EI->_Q( i,i ))*T(-1.,1. ), exp(domain) );
	EllVar3._range() =  Range; 
		
	return EllVar3;
  } 
}

template <class T> 
inline 
EllVar<T> 
log
( const EllVar<T>& EllVar1 )
{
  // EllVar1 is double
  if( !EllVar1._EI && EllVar1._is_constant )
    return std::log( EllVar1._const_val );

  // EllVar1 is interval
  else if( !EllVar1._EI && !EllVar1._is_constant ) 
    return log( EllVar1._Range );

  // Check feasibility of operation
  T domain = EllVar1._range();
  if( Op<T>::l(domain) <= 0.  )
    throw typename EllImg<T>::Exceptions( EllImg<T>::Exceptions::LOG );

  EllImg<T>* EI = EllVar1._EI;
  long i = EI->_current_row;
  long k = EllVar1._RowInd;
#ifdef MC__ELLIMAGE_DEBUG
  EI->_dbugout << std::scientific << std::setprecision(3) << std::right;
  EI->_dbugout << "log starts: i= " << i <<" k= "<< k << std::endl;
  EI->_dbugout << "q \n" << EI->_q << std::endl;
  EI->_dbugout << "Q \n" << EI->_Q << std::endl;
#endif

  // Construct Evar3 to return (increases _current_row)
  EllVar<T> EllVar3( EI, i );
  double c_0, c_1, eta;

  // Compute linear approximation of log( m + r*x ) with x \in [-1,1] using Chebyshev models
  if( EI->options.CHEBUSE )
    std::tie( c_0, c_1, eta ) = EI->_cmodel_linear( log, domain );

  // Compute linear approximation of log( m + r*x ) with x \in [-1,1] using minimax linearisation
  else{
    double m = Op<T>::mid( domain );
    double r = 0.5 * Op<T>::diam( domain );
    double c_sec=0., x_tan=0., c_tan=0.;
    c_sec = (1./2.)*(std::log(m-r)+std::log(m+r));
    c_1   = (1./2.)*(std::log(m+r)-std::log(m-r));
    x_tan = (1./c_1)-(m/r);
    c_tan = std::log(m+r*x_tan)-c_1*x_tan; 
    eta   = std::fabs((c_sec - c_tan)/2.);
    c_0   = c_sec + eta;
    c_1   = c_1/r;
  }

  // Update matrix and variable
  EI->_univupdate( i, k, c_0, c_1, eta ); 
  Op<T>::inter( EllVar3._range(), EI->_q(i)+std::sqrt(EI->_Q( i,i ))*T(-1.,1.), log(domain) );
#ifdef MC__ELLIMAGE_DEBUG
  EI->_dbugout << std::scientific << std::setprecision(3) << std::right;
  EI->_dbugout << "log ends: i= " << i <<" k= "<< k <<std::endl;
  EI->_dbugout << "q \n" ;
  EI->_dbugout << EI->_q <<std::endl;
  EI->_dbugout << "Q \n" ;
  EI->_dbugout << EI->_Q <<std::endl;
#endif   

  return EllVar3;
}

template <class T> 
inline 
EllVar<T> 
sqrt
( const EllVar<T>& EllVar1 )
{
  if( EllVar1._EI == NULL && EllVar1._is_constant ) //  EllVar1 is a double
  {
     return std::sqrt( EllVar1._const_val ) ;
  }
  else if( EllVar1._EI == NULL && ! EllVar1._is_constant ) // EllVar1 is an interval
  {
  	return sqrt( EllVar1._Range ) ;
  }
  else   //EllVar1 is a 'common' EllVar<T>s
  {
	EllImg<T>* EI = EllVar1._EI;
	long i          = EI->_current_row;
	long k          = EllVar1._RowInd;
  
	// Construct Evar3 to return (increases _current_row)
	EllVar<T> EllVar3( EI, i );
	// bound the argument EllVar1 in the current ellipsoid
	T domain = EllVar1._range();
	if( Op<T>::l(domain) < 0.  )
	{	
   		throw typename EllImg<T>::Exceptions( EllImg<T>::Exceptions::SQRT );
  	}
	// Compute linear approximation of sqrt( m + r*x ) with x \in [-1,1] using Chebyshev models
	double c_0, c_1, eta;
	if( EI->options.CHEBUSE )
	{
		std::tie( c_0, c_1, eta ) = EI->_cmodel_linear( sqrt , domain );
	}
	else // Compute linear approximation of sqrt( m + r*x ) with x \in [-1,1] using minimax linearisation
	{
		double m = Op<T>::mid( domain );
		double r = 0.5 * Op<T>::diam( domain );
		double c_sec=0., x_tan=0., c_tan=0.;
		c_sec = (0.5)*(std::sqrt(m-r)+std::sqrt(m+r)); 
		c_1   = (0.5)*(std::sqrt(m+r)-std::sqrt(m-r)); 
		x_tan = (std::pow(r,2) - 4.0*std::pow((c_1),2)*m )/(4.0*std::pow((c_1),2)*r); 
		c_tan = std::sqrt(m+r*x_tan)-c_1*x_tan; 
		eta   = std::fabs((c_sec - c_tan)/2.0); 
		c_0   = c_sec + eta;
		c_1   = c_1 / r ;   
	}								
	// Update matrix 
	EI->_univupdate( i , k , c_0 , c_1 , eta ); 
	// Save range of Variable 
	T Range                  ;
	Op<T>::inter( Range, EI->_q(i)+std::sqrt(EI->_Q( i,i ))*T(-1.,1. ), sqrt(domain) );
	EllVar3._range() =  Range;

	return EllVar3;
  } 
}

template <class T> 
inline 
EllVar<T> 
sqr
( const EllVar<T>& EllVar1 )
{
  if( EllVar1._EI == NULL && EllVar1._is_constant ) //  EllVar1 is a double
  {
     return std::pow( EllVar1._const_val ,2 ) ;
  }
  else if( EllVar1._EI == NULL && ! EllVar1._is_constant ) // EllVar1 is an interval
  {
  	return sqr( EllVar1._Range ) ;
  }
  else   //EllVar1 is a 'common' EllVar<T>s
  {
	EllImg<T>* EI = EllVar1._EI;
	long i        = EI->_current_row;
	long k        = EllVar1._RowInd;
	// Construct Evar3 to return (increases _current_row)
	EllVar<T> EllVar3( EI, i );
	#ifdef MC__ELLIMAGE_DEBUG
	EI->_dbugout << std::scientific << std::setprecision(3) << std::right;
	EI->_dbugout << "sqr starts: i= " << i <<" k= "<< k <<std::endl;
	EI->_dbugout << "q \n" ;
	EI->_dbugout << EI->_q <<std::endl;
	EI->_dbugout << "Q \n" ;
	EI->_dbugout << EI->_Q <<std::endl;
	#endif 
	// bound the argument EllVar1 in the current ellipsoid
	T domain = EllVar1._range();
	// Compute linear approximation of sqr( m + r*x ) with x \in [-1,1] using Chebyshev models
	double c_0, c_1, eta;
	if( EI->options.CHEBUSE )
	{
		std::tie( c_0, c_1, eta ) = EI->_cmodel_linear( sqr , domain );
	}
	else // Compute linear approximation of sqr( m + r*x ) with x \in [-1,1] using minimax linearisation
	{
		double m = Op<T>::mid( domain );
		double r = 0.5 * Op<T>::diam( domain );
		double c_sec=0., x_tan=0., c_tan=0.;
		c_sec = std::pow(m,2)+std::pow(r,2);
		c_1   = 2.0*m*r;
		x_tan = (c_1-2.0*m*r)/(2.0*std::pow(r,2));
		c_tan = std::pow((m+r*x_tan),2)-c_1*x_tan;
		eta   = (c_sec - c_tan)/2.0;
		c_0   = c_sec - eta;
		c_1   = c_1 / r;
	}
	// Update matrix 
	EI->_univupdate( i , k , c_0 , c_1 , eta ); 
	// Save range of Variable 
	T Range                  ;
	Op<T>::inter( Range, EI->_q(i)+std::sqrt(EI->_Q( i,i ))*T(-1.,1. ), sqr(domain) );
	EllVar3._range() =  Range;
	#ifdef MC__ELLIMAGE_DEBUG
	EI->_dbugout << std::scientific << std::setprecision(3) << std::right;
	EI->_dbugout << "sqr ends: i= " << i <<" k= "<< k <<std::endl;
	EI->_dbugout << "q \n" ;
	EI->_dbugout << EI->_q <<std::endl;
	EI->_dbugout << "Q \n" ;
	EI->_dbugout << EI->_Q <<std::endl;
	#endif 
	return EllVar3;
  } 
}

template <class T> 
inline 
EllVar<T> 
pow
( const EllVar<T>& EllVar1 , const int n )
{
  if( EllVar1._EI == NULL && EllVar1._is_constant ) //  EllVar1 is a double
  {
     return std::pow( EllVar1._const_val , n ) ;
  }
  else if( EllVar1._EI == NULL && ! EllVar1._is_constant ) // EllVar1 is an interval
  {
  	return pow( EllVar1._Range , n ) ;
  }
  else   //EllVar1 is a 'common' EllVar<T>s
  {
	EllImg<T>* EI = EllVar1._EI;
	long i          = EI->_current_row;
	long k          = EllVar1._RowInd;
	// Construct Evar3 to return (increases _current_row)
	EllVar<T> EllVar3( EI, i );
	// bound the argument EllVar1 in the current ellipsoid
	T domain = EllVar1._range();
	// Compute linear approximation of pow( m + r*x , n ) with x \in [-1,1] using Chebyshev models
	double c_0, c_1, eta;
	if( EI->options.CHEBUSE )
	{
		std::tie( c_0, c_1, eta ) = EI->_cmodel_linear( n , domain );
	}
	else // Compute linear approximation of log( m + r*x , n ) with x \in [-1,1] using minimax linearisation
	{
		double m = Op<T>::mid( domain );
		double r = 0.5 * Op<T>::diam( domain );
		double c_sec=0., x_tan=0., c_tan=0.;
		double x_tanL, x_tanU, c_tanL, c_tanU;
		if( n % 2 == 0 ) // even power
		{
			c_sec   = (1./2.)*(std::pow((m+r),n)+std::pow((m-r),n));
			c_1     = (1./2.)*(std::pow((m+r),n)-std::pow((m-r),n));
			x_tan   = (std::pow((c_1/(n*r)),1./(n-1))-m)/r;
			c_tan   = std::pow((m+r*x_tan),n)-c_1*x_tan;
			eta     = (c_sec - c_tan)/2.;
			c_0     = c_sec - eta;   	
		}
		else
		{
			if(Op<T>::l( domain ) >= 0 || Op<T>::u( domain ) <= 0)
			{
				c_1     = (1./2.)*(std::pow((m+r),n)-std::pow((m-r),n));
				c_sec   = (1./2.)*(std::pow((m+r),n)+std::pow((m-r),n));
				x_tan   = (sign(m)*std::pow((c_1/(n*r)),1./(n-1))-m)/r;
				c_tan   = std::pow((m+r*x_tan),n)-c_1*x_tan;
				eta     = (c_sec - c_tan)/2.;
				c_0     = c_sec - eta;
			}
			else
			{
				c_1    = (1./2.)*(std::pow((m+r),n)-std::pow((m-r),n));
				x_tanU = (std::pow((c_1/(n*r)),1./(n-1))-m)/r;
				x_tanL = (-1.0*std::pow((c_1/(n*r)),1./(n-1))-m)/r;
				c_tanU = std::pow((m+r*x_tanU),n)-c_1*x_tanU;
				c_tanL = std::pow((m+r*x_tanL),n)-c_1*x_tanL;
				eta    = std::fabs((c_tanL - c_tanU)/2.);
				c_0    = std::fabs(c_tanL - eta);
			}
		}
		c_1 = c_1 / r;
	}
	// Update matrix 
	EI->_univupdate( i , k , c_0 , c_1 , eta );   
	// Save range of Variable 
	T Range                  ;
	Op<T>::inter( Range, EI->_q(i)+std::sqrt(EI->_Q( i,i ))*T(-1.,1. ), pow(domain,n) );
	EllVar3._range() =  Range;

	return EllVar3;
  } 
}

template <class T> 
inline 
EllVar<T> 
cos
( const EllVar<T>& EllVar1 )
{
  if( EllVar1._EI == NULL && EllVar1._is_constant ) //  EllVar1 is a double
  {
     return std::cos( EllVar1._const_val ) ;
  }
  else if( EllVar1._EI == NULL && ! EllVar1._is_constant ) // EllVar1 is an interval
  {
  	return cos( EllVar1._Range ) ;
  }
  else   //EllVar1 is a 'common' EllVar<T>s
  {
	EllImg<T>* EI = EllVar1._EI;
	long i          = EI->_current_row;
	long k          = EllVar1._RowInd;
	// Construct Evar3 to return (increases _current_row)
	EllVar<T> EllVar3( EI, i );
	// bound the argument EllVar1 in the current ellipsoid
	T domain = EllVar1._range();
	// Compute linear approximation of cos( m + r*x ) with x \in [-1,1] using Chebyshev models
	double c_0, c_1, eta;
	std::tie( c_0, c_1, eta ) = EI->_cmodel_linear( cos , domain );
	// Update matrix 
	EI->_univupdate( i , k , c_0 , c_1 , eta ); 
	// Save range of Variable 
	T Range                  ;
	Op<T>::inter( Range, EI->_q(i)+std::sqrt(EI->_Q( i,i ))*T(-1.,1. ), cos(domain) );
	EllVar3._range() =  Range;

	return EllVar3;
  } 
}

template <class T> 
inline 
EllVar<T> 
sin
( const EllVar<T>& EllVar1 )
{
  if( EllVar1._EI == NULL && EllVar1._is_constant ) //  EllVar1 is a double
  {
     return std::sin( EllVar1._const_val ) ;
  }
  else if( EllVar1._EI == NULL && ! EllVar1._is_constant ) // EllVar1 is an interval
  {
  	return sin( EllVar1._Range ) ;
  }
  else   //EllVar1 is a 'common' EllVar<T>s
  {
	EllImg<T>* EI = EllVar1._EI;
	long i          = EI->_current_row;
	long k          = EllVar1._RowInd;
	// Construct Evar3 to return (increases _current_row)
	EllVar<T> EllVar3( EI, i );
	// bound the argument EllVar1 in the current ellipsoid
	T domain = EllVar1._range();
	// Compute linear approximation of sin( m + r*x ) with x \in [-1,1] using Chebyshev models
	double c_0, c_1, eta;
	std::tie( c_0, c_1, eta ) = EI->_cmodel_linear( sin , domain );
	// Update matrix 
	EI->_univupdate( i , k , c_0 , c_1 , eta ); 
	// Save range of Variable 
	T Range                  ;
	Op<T>::inter( Range, EI->_q(i)+std::sqrt(EI->_Q( i,i ))*T(-1.,1. ), sin(domain) );
	EllVar3._range() =  Range;
  
  	return EllVar3;
  } 
}

template <class T> 
inline 
EllVar<T> 
tan
( const EllVar<T>& EllVar1 )
{
  if( EllVar1._EI == NULL && EllVar1._is_constant ) //  EllVar1 is a double
  {
	return std::tan( EllVar1._const_val ) ;
  }
  else if( EllVar1._EI == NULL && ! EllVar1._is_constant ) // EllVar1 is an interval
  {
  	return tan( EllVar1._Range ) ;
  }
  else   //EllVar1 is a 'common' EllVar<T>s
  {
	EllImg<T>* EI = EllVar1._EI;
	long i          = EI->_current_row;
	long k          = EllVar1._RowInd;
	// Construct Evar3 to return (increases _current_row)
	EllVar<T> EllVar3( EI, i );
	// bound the argument EllVar1 in the current ellipsoid
	T domain = EllVar1._range();
	if( Op<T>::l(cos(domain)) <= 0. && Op<T>::u(cos(domain)) >= 0. )
	{	
	throw typename EllImg<T>::Exceptions( EllImg<T>::Exceptions::TAN );
	} 
	// Compute linear approximation of tan( m + r*x ) with x \in [-1,1] using Chebyshev models
	double c_0, c_1, eta;
	std::tie( c_0, c_1, eta ) = EI->_cmodel_linear( tan , domain );
	// Update matrix 
	EI->_univupdate( i , k , c_0 , c_1 , eta ); 
	// Save range of Variable 
	T Range                  ;
	Op<T>::inter( Range, EI->_q(i)+std::sqrt(EI->_Q( i,i ))*T(-1.,1. ), tan(domain) );
	EllVar3._range() =  Range;
  
  	return EllVar3;
  } 
}

template <class T> 
inline 
EllVar<T> 
acos
( const EllVar<T>& EllVar1 )
{
  if( EllVar1._EI == NULL && EllVar1._is_constant ) //  EllVar1 is a double
  {
	return std::acos( EllVar1._const_val ) ;
  }
  else if( EllVar1._EI == NULL && ! EllVar1._is_constant ) // EllVar1 is an interval
  {
  	return acos( EllVar1._Range ) ;
  }
  else   //EllVar1 is a 'common' EllVar<T>s
  {
	EllImg<T>* EI = EllVar1._EI;
	long i          = EI->_current_row;
	long k          = EllVar1._RowInd;
	// Construct Evar3 to return (increases _current_row)
	EllVar<T> EllVar3( EI, i );
	// bound the argument EllVar1 in the current ellipsoid
	T domain = EllVar1._range();
	if( Op<T>::l(domain) < -1. && Op<T>::u(domain) > 1. )
	{	
		throw typename EllImg<T>::Exceptions( EllImg<T>::Exceptions::ACOS );
	} 
	// Compute linear approximation of acos( m + r*x ) with x \in [-1,1] using Chebyshev models
	double c_0, c_1, eta;
	std::tie( c_0, c_1, eta ) = EI->_cmodel_linear( acos , domain );
	// Update matrix 
	EI->_univupdate( i , k , c_0 , c_1 , eta ); 
	// Save range of Variable 
	T Range                  ;
	Op<T>::inter( Range, EI->_q(i)+std::sqrt(EI->_Q( i,i ))*T(-1.,1. ), acos(domain) );
	EllVar3._range() =  Range;
	  
	return EllVar3;
  } 
}

template <class T> 
inline 
EllVar<T> 
asin
( const EllVar<T>& EllVar1 )
{
  if( EllVar1._EI == NULL && EllVar1._is_constant ) //  EllVar1 is a double
  {
	return std::asin( EllVar1._const_val ) ;
  }
  else if( EllVar1._EI == NULL && ! EllVar1._is_constant ) // EllVar1 is an interval
  {
  	return asin( EllVar1._Range ) ;
  }
  else   //EllVar1 is a 'common' EllVar<T>s
  {
	EllImg<T>* EI = EllVar1._EI;
	long i          = EI->_current_row;
	long k          = EllVar1._RowInd;
	// Construct Evar3 to return (increases _current_row)
	EllVar<T> EllVar3( EI, i );
	// bound the argument EllVar1 in the current ellipsoid
	T domain = EllVar1._range();
	if( Op<T>::l(domain) < -1. && Op<T>::u(domain) > 1. )
	{	
		throw typename EllImg<T>::Exceptions( EllImg<T>::Exceptions::ACOS );
	} 
	// Compute linear approximation of asin( m + r*x ) with x \in [-1,1] using Chebyshev models
	double c_0, c_1, eta;
	std::tie( c_0, c_1, eta ) = EI->_cmodel_linear( asin , domain );
	// Update matrix 
	EI->_univupdate( i , k , c_0 , c_1 , eta ); 
	// Save range of Variable 
	T Range                  ;
	Op<T>::inter( Range, EI->_q(i)+std::sqrt(EI->_Q( i,i ))*T(-1.,1. ), asin(domain) );
	EllVar3._range() =  Range;

	return EllVar3;
  } 
}

template <class T> 
inline 
EllVar<T> 
atan
( const EllVar<T>& EllVar1 )
{
  if( EllVar1._EI == NULL && EllVar1._is_constant ) //  EllVar1 is a double
  {
	return std::atan( EllVar1._const_val ) ;
  }
  else if( EllVar1._EI == NULL && ! EllVar1._is_constant ) // EllVar1 is an interval
  {
	return atan( EllVar1._Range ) ;
  }
  else   //EllVar1 is a 'common' EllVar<T>s
  {
	EllImg<T>* EI = EllVar1._EI;
	long i          = EI->_current_row;
	long k          = EllVar1._RowInd;
	// Construct Evar3 to return (increases _current_row)
	EllVar<T> EllVar3( EI, i );
	// bound the argument EllVar1 in the current ellipsoid
	T domain = EllVar1._range();
	// Compute linear approximation of acos( m + r*x ) with x \in [-1,1] using Chebyshev models
	double c_0, c_1, eta;
	std::tie( c_0, c_1, eta ) = EI->_cmodel_linear( atan , domain );
	// Update matrix 
	EI->_univupdate( i , k , c_0 , c_1 , eta ); 
	// Save range of Variable 
	T Range                  ;
	Op<T>::inter( Range, EI->_q(i)+std::sqrt(EI->_Q( i,i ))*T(-1.,1. ), atan(domain) );
	EllVar3._range() =  Range;

	return EllVar3; 
  }
}

template <typename T> inline std::ostream&
operator<<
( std::ostream&out, const EllVar<T>&EllVar )
{
  return out << std::scientific << EllVar.range() << "(index: " << EllVar.index() << ")";
}

template <class T> 
inline std::ostream&
operator<<
( std::ostream&out, const EllImg<T>&E )
{
  return out << static_cast<const Ellipsoid&>( E );
}

} // namespace mc

namespace mc
{

//! @brief Specialization of the structure mc::Op to allow usage of the type mc::Interval for DAG evaluation or as a template parameter in other MC++ classes
template<typename T> struct Op< mc::EllVar<T> >
{
  typedef mc::EllVar<T> EV;
  static EV point( const double c ) { return EV( c ); }
  static EV zeroone() { return EV( mc::Op<T>::zeroone() ); } // ??
  static void I(EV& x, const EV&y) { x = y; } // ?? 
  static double l(const EV& x) { return mc::Op<T>::l(x.range()); }
  static double u(const EV& x) { return mc::Op<T>::u(x.range()); }
  static double abs (const EV& x) { return mc::Op<T>::abs(x.range());  }
  static double mid (const EV& x) { return mc::Op<T>::mid(x.range());  }
  static double diam(const EV& x) { return mc::Op<T>::diam(x.range()); }
  static EV inv (const EV& x) { return mc::inv(x);  }
  static EV sqr (const EV& x) { return mc::sqr(x);  }
  static EV sqrt(const EV& x) { return mc::sqrt(x); }
  static EV log (const EV& x) { return mc::log(x);  }
  static EV xlog(const EV& x) { return x*mc::log(x); }
  static EV fabs(const EV& x) { throw typename mc::EllImg<T>::Exceptions( EllImg<T>::Exceptions::UNDEF ); }
  static EV exp (const EV& x) { return mc::exp(x);  }
  static EV sin (const EV& x) { return mc::sin(x);  }
  static EV cos (const EV& x) { return mc::cos(x);  }
  static EV tan (const EV& x) { return mc::tan(x);  }
  static EV asin(const EV& x) { return mc::asin(x); }
  static EV acos(const EV& x) { return mc::acos(x); }
  static EV atan(const EV& x) { return mc::atan(x); }
  static EV arh (const EV& x, const double k) { return mc::exp(-k/x); }
  static EV erf (const EV& x) { throw typename mc::EllImg<T>::Exceptions( EllImg<T>::Exceptions::UNDEF ); }
  static EV erfc(const EV& x) { throw typename mc::EllImg<T>::Exceptions( EllImg<T>::Exceptions::UNDEF ); }
  static EV fstep(const EV& x) { throw typename mc::EllImg<T>::Exceptions( EllImg<T>::Exceptions::UNDEF ); }
  static EV bstep(const EV& x) { throw typename mc::EllImg<T>::Exceptions( EllImg<T>::Exceptions::UNDEF ); }
  static EV hull(const EV& x, const EV& y) { throw typename mc::EllImg<T>::Exceptions( EllImg<T>::Exceptions::UNDEF ); }
  static EV min (const EV& x, const EV& y) { throw typename mc::EllImg<T>::Exceptions( EllImg<T>::Exceptions::UNDEF ); }
  static EV max (const EV& x, const EV& y) { throw typename mc::EllImg<T>::Exceptions( EllImg<T>::Exceptions::UNDEF ); }
  static EV cheb(const EV& x, const EV& y) { throw typename mc::EllImg<T>::Exceptions( EllImg<T>::Exceptions::UNDEF ); }
  template <typename X, typename Y> static EV pow(const X& x, const Y& y) { return mc::pow(x,y); }
  static EV monomial (const unsigned int n, const EV* x, const int* k) { throw typename mc::EllImg<T>::Exceptions( EllImg<T>::Exceptions::UNDEF ); }
  static bool inter(EV& xIy, const EV& x, const EV& y) { return false; }
  static bool eq(const EV& x, const EV& y) { return x.env() == y.env() && x.index() == y.index() && x.range() == y.range(); }
  static bool ne(const EV& x, const EV& y) { return !eq( x, y ); }
  static bool lt(const EV& x, const EV& y) { throw typename mc::EllImg<T>::Exceptions( EllImg<T>::Exceptions::UNDEF );  }
  static bool le(const EV& x, const EV& y) { throw typename mc::EllImg<T>::Exceptions( EllImg<T>::Exceptions::UNDEF );  }
  static bool gt(const EV& x, const EV& y) { throw typename mc::EllImg<T>::Exceptions( EllImg<T>::Exceptions::UNDEF );  }
  static bool ge(const EV& x, const EV& y) { throw typename mc::EllImg<T>::Exceptions( EllImg<T>::Exceptions::UNDEF );  }
};

} // namespace mc

#endif
