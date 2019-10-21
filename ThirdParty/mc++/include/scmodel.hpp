// Copyright (C) 2009-2016 Benoit Chachuat, Imperial College London.
// All Rights Reserved.
// This code is published under the Eclipse Public License.

/*!
\page page_CHEBYSHEV Chebyshev Model Arithmetic for Factorable Functions
\author Jai Rajyaguru, Mario E. Villanueva, Beno&icirc;t Chachuat

template <typename T> inline void
SCModel<T>::_sdisp1D
( const std::vector<std::map<unsigned,double>>&CVmap,
  const unsigned ndxvar, std::string&CVname, ostream&os )
const
A \f$q\f$th-order Chebyshev model of a Lipschitz-continuous function \f$f:\mathbb{R}^n\to\mathbb{R}\f$ on the domain \f$D\f$, consists of a \f$q^{\rm th}\f$-order multivariate polynomial \f$\mathcal P\f$ in Chebyshev basis , plus a remainder term \f$\mathcal R\f$, so that
\f{align*}
  f({x}) \in \mathcal P({x}-\hat{x}) \oplus \mathcal R, \quad \forall {x}\in D.
\f}
The polynomial part \f$\mathcal P\f$ is propagated symbolically and accounts for functional dependencies. The remainder term \f$\mathcal R\f$, on the other hand, is traditionally computed using interval analysis [Brisebarre & Joldes, 2010]; see figure below. More generally, convex/concave bounds or an ellipsoidal enclosure can be computed for the remainder term of vector-valued functions too. In particular, it can be established that the remainder term has convergence order (no less than) \f$q+1\f$ with respect to the diameter of the domain set \f$D\f$ under mild conditions [Bompadre <I>et al.</I>, 2012].

<CENTER><TABLE BORDER=0>
<TR>
<TD>\image html Chebyshev_model.png</TD>
</TR>
</TABLE></CENTER>

The classes mc::SCModel and mc::SCVar provide an implementation of Chebyshev model arithmetic. We note that mc::SCModel / mc::SCVar is <b>not a verified implementation</b> in the sense that rounding errors are not accounted for in propagating the coefficients in the multivariate polynomial part, which are treated as floating-point numbers.

The implementation of mc::SCModel and mc::SCVar relies on the operator/function overloading mechanism of C++. This makes the computation of Chebyshev models both simple and intuitive, similar to computing function values in real arithmetics or function bounds in interval arithmetic (see \ref page_INTERVAL). Moreover, mc::SCVar can be used as the template parameter of other available types in MC++; for instance, mc::SCVar can be used in order to propagate the underlying interval bounds in mc::McCormick. Likewise, mc::SCVar can be used as the template parameter of the types fadbad::F, fadbad::B and fadbad::T of <A href="http://www.fadbad.com/fadbad.html">FADBAD++</A> for computing Chebyshev models of either the partial derivatives or the Chebyshev coefficients of a factorable function (see \ref sec_CHEBYSHEV_fadbad).

mc::SCModel and mc::SCVar themselves are templated in the type used to propagate bounds on the remainder term. By default, mc::SCModel and mc::SCVar can be used with the non-verified interval type mc::Interval of MC++. For reliability, however, it is strongly recommended to use verified interval arithmetic such as <A href="http://www.ti3.tu-harburg.de/Software/PROFILEnglisch.html">PROFIL</A> (header file <tt>mcprofil.hpp</tt>) or <A href="http://www.math.uni-wuppertal.de/~xsc/software/filib.html">FILIB++</A> (header file <tt>mcfilib.hpp</tt>). As already noted, convex/concave bounds on the remainder term can also be propagated by using the type mc::McCormick of MC++, thereby enabling McCormick-Chebyshev models.

As well as propagating Chebyshev models for factorable functions, mc::SCModel and mc::SCVar provide support for computing bounds on the Chebyshev model range (multivariate polynomial part). We note that computing exact bounds for multivariate polynomials is a hard problem in general. Instead, a number of computationally tractable, yet typically conservative, bounding approaches are implemented in mc::SCModel and mc::SCVar, which include:
- Bounding every monomial term independently and adding these bounds;
- Bounding the first- and diagonal second-order terms exactly and adding bounds for the second-order off-diagonal and higher-order terms computed independently [Lin & Stadtherr, 2007];
- Bounding the terms up to order 2 based on an eigenvalue decomposition of the corresponding Hessian matrix and adding bounds for the higher-order terms computed independently;
- Expressing the multivariate polynomial in Bernstein basis, thereby providing bounds as the minimum/maximum among all Bernstein coefficients [Lin & Rokne, 1995; 1996].
.

Examples of Chebyshev models (blue lines) constructed with mc::SCModel and mc::SCVar are shown on the figure below for the factorable function \f$f(x)=x \exp(-x^2)\f$ (red line) for \f$x\in [-0.5,1]\f$. Also shown on these plots are the interval bounds computed from the Chebyshev models.

<CENTER><TABLE BORDER=0>
<TR>
<TD>\image html CM-1D.png</TD>
</TR>
</TABLE></CENTER>


\section sec_CHEBYSHEV_I How do I compute a Chebyshev model with interval remainder bound of a factorable function?

Suppose we want to compute a 4th-order Chebyshev model for the real-valued function \f$f(x,y)=x\exp(x+y^2)-y^2\f$ with \f$(x,y)\in [1,2]\times[0,1]\f$. For simplicity, bounds on the remainder terms are computed using the default interval type mc::Interval here:

\code
      #include "interval.hpp"
      #include "cmodel.hpp"
      typedef mc::Interval I;
      typedef mc::SCModel<I> CM;
      typedef mc::SCVar<I> CV;
\endcode

First, the number of independent variables in the factorable function (\f$x\f$ and \f$y\f$ here) as well as the order of the Chebyshev model (4th order here) are specified by defining an mc::SCModel object as:

\code
      CM mod( 2, 4 );
\endcode

Next, the variables \f$x\f$ and \f$y\f$ are defined as follows:

\code
      CV X( &mod, 0, I(1.,2.) );
      CV Y( &mod, 1, I(0.,1.) );
\endcode

Essentially, the first line means that <tt>X</tt> is a variable of class mc::SCVar, participating in the Chebyshev model <tt>mod</tt>, belonging to the interval \f$[1,2]\f$, and having index 0 (indexing in C/C++ start at 0 by convention!). The same holds for the Chebyshev variable <tt>Y</tt>, participating in the model <tt>mod</tt>, belonging to the interval \f$[0,1]\f$, and having index 1.

Having defined the variables, a Chebyshev model of \f$f(x,y)=x\exp(x+y^2)-y^2\f$ on \f$[1,2]\times[0,1]\f$ at the mid-point \f$(\frac{3}{2},\frac{1}{2})\f$ is simply computed as:

\code
      CV F = X*exp(X+pow(Y,2))-pow(Y,2);
\endcode

This model can be displayed to the standard output as:

\code
      std::cout << "f Chebyshev model: " << F << std::endl;
\endcode

which produces the following output:

\verbatim
f Chebyshev model: 
   a0    =  8.38199e+00     0  0
   a1    =  1.90755e+00     0  1
   a2    =  3.59621e+00     1  0
   a3    =  7.47482e-01     0  2
   a4    =  9.00782e-01     1  1
   a5    =  6.30186e-01     2  0
   a6    =  1.56945e-01     0  3
   a7    =  3.35238e-01     1  2
   a8    =  1.55141e-01     2  1
   a9    =  6.67468e-02     3  0
   a10   =  3.49519e-02     0  4
   a11   =  6.58449e-02     1  3
   a12   =  6.04330e-02     2  2
   a13   =  1.80397e-02     3  1
   a14   =  5.41191e-03     4  0
   R     =  [ -2.09182e+00 :  2.22652e+00 ]
   B     =  [ -1.02564e+01 :  3.93973e+01 ]
\endverbatim

<tt>a0</tt>,...,<tt>a14</tt> refer to the coefficients of the monomial terms in the Chebyshev model, with the corresponding variable orders given in the subsequent columns. The remainder term as well as the Chebyshev model range estimator are reported next.

Other operations involve retreiving the remainder bound, centering the remainder term in a Chebyshev model, or computing the value of its polynomial part at a given point:

\code
      I B = F.B();
      F.C();
      double x[2] = { 0.5, 1.5 };
      double Pval = F.P( x );
\endcode

See the documentations of mc::SCModel and mc::SCVar for a complete list of member functions. 


\section sec_CHEBYSHEV_fct Which functions are overloaded for Chebyshev model arithmetic?

mc::SCVar overloads the usual functions <tt>exp</tt>, <tt>log</tt>, <tt>sqr</tt>, <tt>sqrt</tt>, <tt>pow</tt>, <tt>inv</tt>, <tt>cos</tt>, <tt>sin</tt>, <tt>tan</tt>, <tt>acos</tt>, <tt>asin</tt>, <tt>atan</tt>. Unlike mc::Interval and mc::McCormick, the functions <tt>min</tt>, <tt>max</tt> and <tt>fabs</tt> are not overloaded in mc::SCVar as they are nonsmooth. Moreover, mc::SCVar defines the following functions:
- <tt>inter(x,y,z)</tt>, computing a Chebyshev model of the intersection \f$x = y\cap z\f$ of two Chebyshev models and returning true/false if the intersection is nonempty/empty. With Chebyshev models \f$\mathcal P_y\oplus\mathcal R_y\f$ and \f$\mathcal P_z\oplus\mathcal R_z\f$, this intersection is computed as follows:
\f{align*}
  \mathcal P_{x} =\ & (1-\eta) \mathcal P_y^{\rm C} + \eta \mathcal P_z^{\rm C}\\
  \mathcal R_{x} =\ & [\mathcal R_y^{\rm C}\oplus\eta\mathcal{B}(\mathcal P_y^{\rm C}-\mathcal P_z^{\rm C})] \cap [\mathcal R_z^{\rm C}\oplus (1-\eta)\mathcal{B}(\mathcal P_z^{\rm C}-\mathcal P_y^{\rm C})]\,.
\f}
with \f$\mathcal{B}(\cdot)\f$ the Chebyshev model range bounder, and \f$\eta\f$ a real scalar in \f$[0,1]\f$. Choosing \f$\eta=1\f$ amounts to setting the polynomial part \f$\mathcal P_{x}\f$ as \f$\mathcal P_y\f$, whereas \f$\eta=0\f$ sets \f$\mathcal P_{x}\f$ as \f$\mathcal P_z\f$. The parameter \f$\eta\f$ can be defined in mc::SCModel::Options::REF_POLY.
- <tt>hull(x,y)</tt>, computing a Chebyshev model of the union \f$x = y\cup z\f$ of two Chebyshev models. With Chebyshev models \f$\mathcal P_y\oplus\mathcal R_y\f$ and \f$\mathcal P_z\oplus\mathcal R_z\f$, this union is computed as follows:
\f{align*}
  \mathcal P_{x} =\ & (1-\eta) \mathcal P_y^{\rm C} + \eta \mathcal P_z^{\rm C}\\
  \mathcal R_{x} =\ & {\rm hull}\{\mathcal R_y^{\rm C}\oplus\eta\mathcal{B}(\mathcal P_y^{\rm C}-\mathcal P_z^{\rm C}), \mathcal R_z^{\rm C}\oplus (1-\eta)\mathcal{B}(\mathcal P_z^{\rm C}-\mathcal P_y^{\rm C})\}\,.
\f}
with \f$\mathcal{B}(\cdot)\f$ and \f$\eta\f$ as previously.


\section sec_CHEBYSHEV_opt How are the options set for the computation of a Chebyshev model?

The class mc::SCModel has a public member called mc::SCModel::options that can be used to set/modify the options; e.g.,

\code
      model.options.BOUNDER_TYPE = CM::Options::EIGEN;
      model.options.SCALE_VARIABLES = true;
\endcode

The available options are the following:

<TABLE border="1">
<CAPTION><EM>Options in mc::SCModel::Options: name, type and description</EM></CAPTION>
     <TR><TH><b>Name</b>  <TD><b>Type</b><TD><b>Default</b>
         <TD><b>Description</b>
     <TR><TH><tt>BOUNDER_TYPE</tt> <TD><tt>mc::SCModel::Options::BOUNDER</tt> <TD>mc::SCModel::Options::LSB
         <TD>Chebyshev model range bounder.
     <TR><TH><tt>BOUNDER_ORDER</tt> <TD><tt>unsigned int</tt> <TD>0
         <TD>Order of Bernstein polynomial for Chebyshev model range bounding, when mc::SCModel::options::BOUNDER_TYPE = mc::SCModel::options::BERNSTEIN is selected. Only values greater than the actual Chebyshev model order are accounted for; see [Lin & Rokne, 1996].
     <TR><TH><tt>REF_POLY</tt> <TD><tt>double</tt> <TD>0.
         <TD>Scalar in \f$[0,1]\f$ related to the choice of the polynomial part in the overloaded functions mc::inter and mc::hull (see \ref sec_CHEBYSHEV_fct). A value of 0. amounts to selecting the polynomial part of the left operand, whereas a value of 1. selects the right operand.
     <TR><TH><tt>DISPLAY_DIGITS</tt> <TD><tt>unsigned int</tt> <TD>5
         <TD>Number of digits in output stream for Chebyshev model coefficients.
</TABLE>


\section sec_CM_err Errors What errors can I encounter during computation of a Chebyshev model?

Errors are managed based on the exception handling mechanism of the C++ language. Each time an error is encountered, a class object of type mc::SCModel::Exceptions is thrown, which contains the type of error. It is the user's responsibility to test whether an exception was thrown during the computation of a Chebyshev model, and then make the appropriate changes. Should an exception be thrown and not caught by the calling program, the execution will abort.

Possible errors encountered during the computation of a Chebyshev model are:

<TABLE border="1">
<CAPTION><EM>Errors during the Computation of a Chebyshev Model</EM></CAPTION>
     <TR><TH><b>Number</b> <TD><b>Description</b>
     <TR><TH><tt>1</tt> <TD>Division by zero
     <TR><TH><tt>2</tt> <TD>Failed to compute eigenvalue decomposition in range bounder SCModel::Options::EIGEN
     <TR><TH><tt>3</tt> <TD>Failed to compute the maximum gap between a univariate term and its Bernstein model
     <TR><TH><tt>-1</tt> <TD>Number of variable in Chebyshev model must be nonzero
     <TR><TH><tt>-2</tt> <TD>Failed to construct Chebyshev variable
     <TR><TH><tt>-3</tt> <TD>Chebyshev model bound does not intersect with bound in template parameter arithmetic
     <TR><TH><tt>-4</tt> <TD>Operation between Chebyshev variables linked to different Chebyshev models
     <TR><TH><tt>-5</tt> <TD>Maximum size of Chebyshev model reached (monomials indexed as unsigned int)
     <TR><TH><tt>-33</tt> <TD>Feature not yet implemented in mc::SCModel
</TABLE>

Moreover, exceptions may be thrown by the template parameter class itself.


\section sec_CM_refs References

- Brisebarre, N., and M. Joldes, <A href="http://hal.archives-ouvertes.fr/docs/00/48/17/37/PDF/RRLIP2010-13.pdf">Chebyshev Interpolation Polynomial-based Tools for Rigorous Computing</A>, <i>Research Report No RR2010-13</i>, Ecole Normale Sup&eaccute;rieure de Lyon, Unit&eaccute; Mixte de Recherche CNRS-INRIA-ENS LYON-UCBL No 5668, 2010
.
*/

#ifndef MC__SCMODEL_H
#define MC__SCMODEL_H

#include "spolymodel.hpp"
#include "mcop.hpp"

#undef  MC__SCMODEL_DEBUG
#undef  MC__SCMODEL_DEBUG_SCALE
#define MC__SCMODEL_CHECK
#undef  MC__SCMODEL_CHECK_PMODEL
#undef  MC__SCVAR_DEBUG_EXP
#undef  MC__SCVAR_DEBUG_BERNSTEIN

//#undef MC__SCVAR_FABS_SQRT
//#undef MC__SCVAR_FORCE_REM_DERIV
//#undef MC__SCVAR_SPARSE_PRODUCT_FULL

namespace mc
{

template <typename T> class SCVar;

//! @brief C++ class for the computation of sparse Chebyshev models for factorable function: environment
////////////////////////////////////////////////////////////////////////
//! mc::SCModel is a C++ class for definition of Chebyshev model
//! environment in sparse format. Propagation of Chebyshev models for
//! factorable functions is via the C++ class mc::SCVar. The template
//! parameter corresponds to the type
//! used to propagate the remainder bound.
////////////////////////////////////////////////////////////////////////
template <typename T>
class SCModel
////////////////////////////////////////////////////////////////////////
{
  friend class SCVar<T>;
  template <typename U> friend class SCModel;
  template <typename U> friend SCVar<U> pow
    ( const SCVar<U>&, const int );
  typedef std::pair< unsigned, std::map< unsigned, unsigned > > t_expmon;
  typedef std::map< t_expmon, double > t_coefmon;

protected:
  //! @brief Order of polynomial model
  unsigned _maxord;

  //! @brief Number of variables in polynomial model
  unsigned _nvar;

  //! @brief Bounds on model variables
  std::vector<T> _bndvar;

  //! @brief Reference points for the model variables
  std::vector<double> _refvar;

  //! @brief Scaling factors of the model variables
  std::vector<double> _scalvar; 

  //! @brief Coefficients in Chebyshev interpolant of univariate functions
  std::vector<double> _coefinterp;
      
  //! @brief Resize the model variable data containers
  void _resize
    ( const unsigned ivar )
    { if( ivar < _nvar ) return; _nvar = ivar+1;
      _bndvar.resize( _nvar ); _refvar.resize( _nvar ); _scalvar.resize( _nvar ); }
      
  //! @brief Resize the model variable data containers
  void _set
    ( const unsigned i, const T&X )
    { _resize( i );
      _bndvar[i] = X; _refvar[i] = Op<T>::mid(X); _scalvar[i] = 0.5*Op<T>::diam(X); }

  //! @brief Resize and return a pointer to the Chebyshev coefficient interpolant array
  double* _resize_coefinterp
    ()
    { _coefinterp.resize( _maxord + options.INTERP_EXTRA + 1 );
      return _coefinterp.data(); }

public:
  /** @addtogroup SCHEBYSHEV Chebyshev Model Arithmetic for Factorable Functions
   *  @{
   */
  //! @brief Constructor of Sparse Chebyshev model environment for maximal order <tt>maxord</tt>
  SCModel
    ( const unsigned maxord=3, const unsigned nvarres=0 )
    : _maxord( maxord ), _nvar( 0 )
    { _coefinterp.reserve( maxord+1 ); if( !nvarres ) return;
      _bndvar.reserve( nvarres ); _refvar.reserve( nvarres ); _scalvar.reserve( nvarres ); }

  //! @brief Destructor of Sparse Chebyshev model environment
  ~SCModel()
    {}

  //! @brief Number of variables in polynomial model
  unsigned nvar
    ()
    const
    { return _nvar; };

  //! @brief Maximal order of polynomial model
  unsigned maxord
    ()
    const
    { return _maxord; };

  //! @brief Const reference to the bounds of the model variables
  const std::vector<T>& bndvar() const
    { return _bndvar; }

  //! @brief Const reference to reference points for the model variables
  const std::vector<double>& refvar() const
    { return _refvar; }

  //! @brief Const reference to the scaling factors of the model variables
  const std::vector<double>& scalvar() const
    { return _scalvar; }

  //! @brief Get Chebyshev basis functions in U arithmetic for variable array <a>X</a>
  template <typename U> U** get_basis
    ( const unsigned maxord, const U*bndvar, const bool scaled=false ) const;

  //! @brief Get Chebyshev monomial bounds in U arithmetic for variable array <a>X</a> and monomial indexes in <a>ndxmon</a>
  template <typename U> void get_bndmon
    ( std::map<t_expmon,U>&bndmon, const U*bndvar, const bool scaled=false ) const;
/*
  //! @brief Polynomial range bounder using specified bounder <a>type</a> with basis functions <a>bndbasis</a> in U arithmetic and monomial coefficients <a>coefmon</a> in C arithmetic
  template <typename C, typename U> U get_bound
    ( const C*coefmon, const U*const*bndbasis, const U*bndrem, const int type,
      const std::set<unsigned>&ndxmon=std::set<unsigned>() )
    { return( bndrem? _polybound( coefmon, bndbasis, type, ndxmon ) + *bndrem:
                      _polybound( coefmon, bndbasis, type, ndxmon ) ); }
*/
  //! @brief Exceptions of mc::SCModel
  class Exceptions
  {
  public:
    //! @brief Enumeration type for SCModel exception handling
    enum TYPE{
      DIV=1,	//!< Division by zero scalar
      INV,	//!< Inverse operation with zero in range
      LOG,	//!< Log operation with non-positive numbers in range
      SQRT,	//!< Square-root operation with negative numbers in range
      ACOS,	//!< Sine/Cosine inverse operation with range outside [-1,1]
      EIGEN,	//!< Failed to compute eigenvalue decomposition in range bounder SCModel::Options::EIGEN
      INIT=-1,	//!< Failed to construct Chebyshev variable
      INCON=-2, //!< Chebyshev model bound does not intersect with bound in template parameter arithmetic
      SCMODEL=-3,//!< Operation between Chebyshev variables linked to different Chebyshev models
      INTERNAL = -4,//!< Internal error
      UNDEF=-33 //!< Feature not yet implemented in mc::SCModel
    };
    //! @brief Constructor for error <a>ierr</a>
    Exceptions( TYPE ierr ) : _ierr( ierr ){}
    //! @brief Error flag
    int ierr(){ return _ierr; }
    //! @brief Error description
    std::string what(){
      switch( _ierr ){
      case DIV:
        return "mc::SCModel\t Division by zero scalar";
      case INV:
        return "mc::SCModel\t Inverse operation with zero in range";
      case LOG:
        return "mc::SCModel\t Log operation with non-positive numbers in range";
      case SQRT:
        return "mc::SCModel\t Square-root operation with negative numbers in range";
      case ACOS:
        return "mc::SCModel\t Sine/Cosine inverse operation with range outside [-1,1]";
      case EIGEN:
        return "mc::SCModel\t Range bounder with eigenvalue decomposition failed";
      case INIT:
        return "mc::SCModel\t Chebyshev variable initialization failed";
      case INCON:
        return "mc::SCModel\t Inconsistent bounds with template parameter arithmetic";
      case SCMODEL:
        return "mc::SCModel\t Operation between Chebyshev variables in different Chebyshev model environment not allowed";
      case UNDEF:
        return "mc::SCModel\t Feature not yet implemented in mc::SCModel class";
      case INTERNAL:
      default:
        return "mc::SCModel\t Internal error";
      }
    }

  private:
    TYPE _ierr;
  };

  //! @brief Options of mc::SCModel
  struct Options
  {
    //! @brief Constructor of mc::SCModel::Options
    Options():
      INTERP_EXTRA(0), BOUNDER_TYPE(LSB), BOUNDER_ORDER(0), MIXED_IA(true),
      REF_POLY(0.), DISPLAY_DIGITS(5)
      {}
    //! @brief Copy constructor of mc::SCModel::Options
    template <typename U> Options
      ( U&options )
      : INTERP_EXTRA( options.INTERP_EXTRA ),
        BOUNDER_TYPE( options.BOUNDER_TYPE ),
        BOUNDER_ORDER( options.BOUNDER_ORDER ),
        MIXED_IA( options.MIXED_IA ),
        REF_POLY(options.REF_POLY),
	DISPLAY_DIGITS(options.DISPLAY_DIGITS)
      {}
    //! @brief Assignment of mc::SCModel::Options
    template <typename U> Options& operator =
      ( U&options ){
        INTERP_EXTRA     = options.INTERP_EXTRA;
        BOUNDER_TYPE     = (BOUNDER)options.BOUNDER_TYPE;
        BOUNDER_ORDER    = options.BOUNDER_ORDER;
        MIXED_IA         = options.MIXED_IA;
        REF_POLY         = options.REF_POLY;
	DISPLAY_DIGITS   = options.DISPLAY_DIGITS;
        return *this;
      }
    //! @brief Chebyshev model range bounder option
    enum BOUNDER{
      NAIVE=0,	//!< Naive polynomial range bounder
      LSB,	//!< Lin & Stadtherr range bounder
      EIGEN,	//!< Eigenvalue decomposition-based bounder
      BERNSTEIN //!< Bernstein range bounder
    };
    //! @brief Extra terms in chebyshev interpolation of univariates: 0-Chebyshev interpolation of order MAXORD; extra terms allow approximation of Chebyshev truncated series
    unsigned INTERP_EXTRA;
    //! @brief Chebyshev model range bounder - See \ref sec_CHEBYSHEV_opt
    BOUNDER BOUNDER_TYPE;
    //! @brief Order of Bernstein polynomial for Chebyshev model range bounding (no less than Chebyshev model order!). Only if mc::SCModel::options::BOUNDER_TYPE is set to mc::SCModel::options::BERNSTEIN.
    unsigned BOUNDER_ORDER;
    //! @brief Array of Chebyshev model range bounder names (for display)
    static const std::string BOUNDER_NAME[5];
    //! @brief Whether to intersect internal bounds with underlying bounds in T arithmetics
    bool MIXED_IA;
    //! @brief Scalar in \f$[0,1]\f$ related to the choice of the polynomial part in the overloaded functions mc::inter and mc::hull (see \ref sec_CHEBYSHEV_fct). A value of 0. amounts to selecting the polynomial part of the left operand, whereas a value of 1. selects the right operand.
    double REF_POLY;
    //! @brief Number of digits in output stream for Chebyshev model coefficients.
    unsigned DISPLAY_DIGITS;
  } options;
  /** @} */

private:
  //! @brief Get Chebyshev basis functions in U arithmetic for variable <a>bndvar</a>
  template <typename U> static U* _get_bndpow
    ( const unsigned maxord, const U&bndvar, const double ref, const double scal );

  //! @brief Get Chebyshev basis functions in U arithmetic for [-1,1] scaled variable <a>bndvar</a>
  template <typename U> static U* _get_bndpow
    ( const unsigned maxord, const U&bndvar );

  //! @brief Prototype real-valued function for interpolation
  typedef double (puniv)
    ( const double x );

  //! @brief Construct Chebyshev interpolating polynomial coefficient <a>coefmon</a> for univariate <a>f</a>
  static void _interpolation
    ( double*coefmon, const unsigned maxord, const T&X, puniv f );

  //! @brief Apply Chebyshev composition to variable <a>CVI</a> using the coefficients <a>coefmon</a> of the outer function
  template <typename U> static SCVar<T> _composition
    ( const U* coefouter, const unsigned maxord, const SCVar<T>& CVinner );

  //! @brief Recursive calculation of nonnegative integer powers
  SCVar<T> _intpow
    ( const SCVar<T>&CV, const int n ) const;

  //! @brief Polynomial range bounder - Naive approach
  template <typename C, typename U> U _polybound_naive
    ( const std::map< t_expmon, C >& coefmon, const U*const*bndbasis,
      const unsigned minord=0 ) const;

  //! @brief Polynomial range bounder - Lin & Stadtherr approach
  template <typename C, typename U> U _polybound_LSB
    ( const std::map< t_expmon, C >& coefmon, const U*const*bndbasis ) const;

  //! @brief Polynomial range bounder - eigenvalue decomposition approach
  template <typename C, typename U> U _polybound_eigen
    ( const std::map< t_expmon, C >& coefmon, const U*const*bndbasis ) const;

  //! @brief Polynomial range bounder - Bernstein approach
  template <typename C, typename U> U _polybound_bernstein
    ( const std::map< t_expmon, C >& coefmon, const U*const*bndbasis );

  //! @brief Compute Bernstein coefficient for variable with exponents <tt>jexp</tt>, given coefficients in Chebyshev form <tt>coefmon</tt>, maximum order <tt>maxord</tt> and transformation coefficients <a>trmat</a>
  template <typename C> C _coef_bernstein
    ( const std::map< t_expmon, C >& coefmon, const unsigned*jexp, const double*trmat ) const;

  //! @brief Compute Bernstein tranformation coefficient into Bernstein basis function <a>j</a> with order <a>n</a> from Chebyshev basis function <a>k</a>
  double _transform_bernstein
    ( const unsigned j, const unsigned k, const unsigned n ) const;

  //! @brief Polynomial range bounder using specified bounder <a>type</a> with basis functions <a>bndbasis</a> in U arithmetic and monomial coefficients <a>coefmon</a> in C arithmetic
  template <typename C, typename U> U _polybound
    ( const std::map< t_expmon, C >& coefmon, const U*const*bndbasis,
      const int type );

  //! @brief Scaling of Chebyshev coefficient maps
  void _sscal1D
    ( const t_coefmon& coefmon0, const double&coefscal,
      t_coefmon& coefmon ) const;

  //! @brief Recursive product of univariate Chebyshev polynomials
  void _sprod1D
    ( const std::vector<t_coefmon>& CV1vec,
      const std::vector<t_coefmon>& CV2vec,
      t_coefmon& coefmon, double&coefrem,
      const unsigned ndxvar ) const;

  //! @brief Lifting of Chebyshev coefficient maps
  void _slift1D
    ( const t_coefmon& coefmon0, const double&dscal,
      t_coefmon& coefmon ) const;

  //! @brief Lifting of Chebyshev coefficient maps
  void _slift1D
    ( const t_coefmon& coefmon0, const double&dscal,
      t_coefmon& coefmon, double&coefrem,
      const unsigned ndxvar, const unsigned ndxord ) const;

  //! @brief Lifting of Chebyshev coefficient maps
  void _slift1D
    ( const t_coefmon& coefmon0, double&coefrem ) const;

  //! @brief Display of recursive univariate Chebyshev polynomials
  void _sdisp1D
    ( const std::vector<SCVar<T>>& coefmon, const unsigned ndxvar,
      const std::string&name="", std::ostream&os=std::cout ) const;

  //! @brief Display of recursive univariate Chebyshev polynomials
  void _sdisp1D
    ( const std::vector<t_coefmon>& coefmon, const unsigned ndxvar,
      const std::string&name="", std::ostream&os=std::cout ) const;

  //! @brief Display of recursive univariate Chebyshev polynomials
  void _sdisp1D
    ( const t_coefmon& coefmon, const std::string&name="",
      std::ostream&os=std::cout ) const;

  //! @brief Build 1D vector of Chebyshev coefficients
  void _svec1Dfull
    ( const unsigned ndxvar, typename t_coefmon::const_iterator it,
      std::vector<SCVar<T>>& vec ) const;

  //! @brief Build 1D vector of Chebyshev coefficients
  void _svec1D
    ( const unsigned ndxvar, typename t_coefmon::const_iterator it,
      std::vector<t_coefmon>& vec ) const;

  //! @brief Product of multivariate Chebyshev polynomials in sparse format
  void _sprod
    ( const SCVar<T>&CV1, const SCVar<T>&CV2, t_coefmon& coefmon,
      double&coefrem ) const;

  //! @brief Squaring of multivariate Chebyshev polynomials in sparse format
  void _ssqr
    ( const SCVar<T>&CV, t_coefmon& coefmon, double&coefrem ) const;
};

template <typename T> const std::string SCModel<T>::Options::BOUNDER_NAME[5]
  = { "NAIVE", "LSB", "EIGEN", "BERNSTEIN", "HYBRID" };

//! @brief C++ class for Chebyshev model computation of factorable function - Chebyshev model propagation
////////////////////////////////////////////////////////////////////////
//! mc::SCVar is a C++ class for propagation of Chebyshev models through
//! factorable functions. The template parameter corresponds to the
//! type used in computing the remainder bound.
////////////////////////////////////////////////////////////////////////
template <typename T>
class SCVar: public SPolyVar<T>
////////////////////////////////////////////////////////////////////////
{
  template <typename U> friend class SCVar;
  template <typename U> friend class SCModel;

  template <typename U> friend SCVar<U> operator-
    ( const SCVar<U>& );
  template <typename U> friend SCVar<U> operator*
    ( const SCVar<U>&, const SCVar<U>& );
  template <typename U> friend std::ostream& operator<<
    ( std::ostream&, const SCVar<U>& );

  template <typename U> friend U funcptr
    ( const U, const unsigned  );
  template <typename U> friend void interpolation
    ( double*, const SCVar<U>&, const unsigned  );
  template <typename U> friend SCVar<U> composition
    ( const double*, const SCVar<U>& );

  template <typename U> friend SCVar<U> inv
    ( const SCVar<U>& );
  template <typename U> friend SCVar<U> sqr
    ( const SCVar<U>& );
  template <typename U> friend SCVar<U> sqrt
    ( const SCVar<U>& );
  template <typename U> friend SCVar<U> exp
    ( const SCVar<U>& );
  template <typename U> friend SCVar<U> log
    ( const SCVar<U>& );
  template <typename U> friend SCVar<U> xlog
    ( const SCVar<U>& );
  template <typename U> friend SCVar<U> pow
    ( const SCVar<U>&, const int );
  template <typename U> friend SCVar<U> pow
    ( const SCVar<U>&, const double );
  template <typename U> friend SCVar<U> pow
    ( const double, const SCVar<U>& );
  template <typename U> friend SCVar<U> pow
    ( const SCVar<U>&, const SCVar<U>& );
  template <typename U> friend SCVar<U> monomial
    ( const unsigned int, const SCVar<U>*, const int* );
  template <typename U> friend SCVar<U> cheb
    ( const SCVar<U>&, const unsigned );
  template <typename U> friend SCVar<U> cos
    ( const SCVar<U>& );
  template <typename U> friend SCVar<U> sin
    ( const SCVar<U>& );
  template <typename U> friend SCVar<U> tan
    ( const SCVar<U>& );
  template <typename U> friend SCVar<U> acos
    ( const SCVar<U>& );
  template <typename U> friend SCVar<U> asin
    ( const SCVar<U>& );
  template <typename U> friend SCVar<U> atan
    ( const SCVar<U>& );
  template <typename U> friend SCVar<U> fabs
    ( const SCVar<U>& );
  template <typename U> friend SCVar<U> hull
    ( const SCVar<U>&, const SCVar<U>& );
  template <typename U> friend bool inter
    ( SCVar<U>&, const SCVar<U>&, const SCVar<U>& );

public:
  typedef std::pair< unsigned, std::map< unsigned, unsigned > > t_expmon;
  typedef std::map< t_expmon, double > t_coefmon;

  using SPolyVar<T>::_coefmon;
  using SPolyVar<T>::_bndrem;
  using SPolyVar<T>::_bndT;
  using SPolyVar<T>::_set_bndT;
  using SPolyVar<T>::_unset_bndT;
  using SPolyVar<T>::_bndpol;
  using SPolyVar<T>::_set_bndpol;
  using SPolyVar<T>::_unset_bndpol;
  using SPolyVar<T>::_reinit;
  using SPolyVar<T>::_set;
  using SPolyVar<T>::set;
  using SPolyVar<T>::bound;
  using SPolyVar<T>::_center;

private:
  //! @brief Pointer to Chebyshev model environment
  SCModel<T> *_CM;

  //! @brief Original bounds on variable <tt>ivar</tt>
  const T& _bndvar
    ( const unsigned ivar ) const
    { return _CM->_bndvar[ivar]; };

  //! @brief Reference point for variable <tt>ivar</tt> in Chebyshev model
  double _refvar
    ( const unsigned ivar ) const
    { return _CM->_refvar[ivar]; };

  //! @brief Scaling for variable <tt>ivar</tt> in Cheyshev model
  double _scalvar
    ( const unsigned ivar ) const
    { return _CM->_scalvar[ivar]; };

  //! @brief Set Chebyshev variable with index <a>ix</a> (starting from 0) and bounded by <a>X</a>
  SCVar<T>& _set
    ( const unsigned ivar, const T&X, const bool updMod=true );

  //! @brief Product of multivariate Chebyshev polynomials in sparse format
  void _sprod
    ( const SCVar<T>& CV1, const SCVar<T>& CV2, t_coefmon& coefmon,
      double& coefrem ) const
    { return _CM->_sprod( CV1, CV2, coefmon, coefrem ); }

  //! @brief Squaring of multivariate Chebyshev polynomials in sparse format
  void _ssqr
    ( const SCVar<T>& CV, t_coefmon& coefmon, double& coefrem ) const
    { return _CM->_ssqr( CV, coefmon, coefrem ); }

  //! @brief Array of Chebyshev interpolant coefficients
  double* _coefinterp() const
    { return _CM->_resize_coefinterp(); };

public:
  /** @addtogroup CHEBYSHEV Chebyshev Model Arithmetic for Factorable Functions
   *  @{
   */
  //! @brief Get pointer to linked Chebyshev model environment
  SCModel<T>* env() const
    { return _CM; }

  //! @brief Maximal order of model environment
  unsigned maxord
    ()
    const
    { return( _CM? _CM->_maxord: 0 ); };

  //! @brief Number of variables in model environment
  unsigned nvar
    ()
    const
    { return( _CM? _CM->_nvar: 0 ); };

  //! @brief Constructor of Chebyshev variable for a real scalar
  SCVar
    ( SCModel<T>*CM );

  //! @brief Constructor of Chebyshev variable for a real scalar
  SCVar
    ( const double d=0., SCModel<T>*CM=0 );

  //! @brief Constructor of Chebyshev variable for a remainder bound
  SCVar
    ( const T&B, SCModel<T>*CM=0 );

  //! @brief Constructor of Chebyshev variable with index <a>ix</a> (starting from 0) and bounded by <a>X</a>
  SCVar
    ( SCModel<T>*CM, const unsigned ix, const T&X );
/*
  //! @brief Copy constructor of Chebyshev variable in different Chebyshev model environment (with implicit type conversion)
  template <typename U> SCVar
    ( SCModel<T>*&CM, const SCVar<U>&CV );

  //! @brief Copy constructor of Chebyshev variable in different Chebyshev model environment (with explicit type conversion as given by class member function <a>method</a>)
  template <typename U> SCVar
    ( SCModel<T>*&CM, const SCVar<U>&CV, const T& (U::*method)() const );

  //! @brief Copy constructor of Chebyshev variable in different Chebyshev model environment (with explicit type conversion as given by non-class member function <a>method</a>)
  template <typename U> SCVar
    ( SCModel<T>*&CM, const SCVar<U>&CV, T (*method)( const U& ) );
*/
  //! @brief Copy constructor of Chebyshev variable
  SCVar
    ( const SCVar<T>&CV )
    : SPolyVar<T>( CV ), _CM( CV._CM )
    {
#ifdef  MC__SCMODEL_CHECK_PMODEL
      if( _CM != dynamic_cast< SCModel<T>* >( SPolyVar<T>::_CM ) ) assert( false );
#endif
    }

  //! @brief Destructor of Chebyshev variable
  ~SCVar()
    {}

  //! @brief Set Chebyshev variable with index <a>ix</a> (starting from 0) and bounded by <a>X</a>
  SCVar<T>& set
    ( SCModel<T>*CM, const unsigned ix, const T&X )
    { set( CM ); _set( ix, X ); return *this; }

  //! @brief Set environment in Chebyshev model
  SCVar<T>& set
    ( SCModel<T>*CM )
    { _CM = CM; return *this; }
/*
  //! @brief Set Chebyshev model environment in Chebyshev variable to <tt>CM</tt>
  SCVar<T>& set
    ( SCModel<T>*CM, const bool reset=false )
    { if( CM != _CM ){ _CM = CM; SPolyVar<T>::set( _CM, reset ); } return *this; }
*/
  //! @brief Retreive bound on variable using default bounder in U arithmetic
  template <typename U> U bound
    ( const U*const*bndbasis, const U&bndrem ) const
    { return _polybound( bndbasis ) + bndrem; }

  //! @brief Retreive bound on variable using bounder <a>type</a> in U arithmetic
  template <typename U> U bound
    ( const U*const*bndbasis, const U&bndrem, const int type ) const
    { return _polybound( bndbasis, type ) + bndrem; }

  //! @brief Retreive bound on all terms with (total) order <tt>minord</tt> or higher in polynomial part
  T bndord
    ( const unsigned minord )
    const
    { if( !_CM ) return !minord && !_coefmon.empty() && !_coefmon.begin()->first.first? _coefmon.begin()->second: 0.;
      return _CM->_polybound_naive( _coefmon, (const T*const*)0, minord ); }

  //! @brief Evaluate polynomial part at <tt>x</tt>
  double polynomial
    ( const double*x ) const;

  //! @brief Shortcut to mc::SCVar::polynomial
  double P
    ( const double*x ) const
    { return polynomial( x ); }

  //! @brief Return new Chebyshev variable with same multivariate polynomial part but zero remainder
  SCVar<T> polynomial
    ()
    const
    { SCVar<T> var = *this; var._bndrem = 0.; return var; }

  //! @brief Shortcut to mc::SCVar::polynomial
  SCVar<T> P
    ()
    const
    { return polynomial(); }

  //! @brief Center remainder term of Chebyshev variable
  SCVar<T>& center
    ()
    { _center(); return *this; }

  //! @brief Shortcut to mc::SCVar::center
  SCVar<T>& C
    ()
    { return center(); }

  //! @brief Get coefficient of constant term in Chebyshev variable. The value of this coefficient is reset to 0 if <tt>reset=true</tt>, otherwise it is left unmodified (default).
  double constant
    ( const bool reset=false );

  //! @brief Get coefficients of linear term for variable <tt>ivar</tt> in Chebyshev variable. The value of this coefficient is reset to 0 if <tt>reset=true</tt>, otherwise it is left unmodified (default).
  double linear
    ( const unsigned ivar, const bool reset=false );

  //! @brief Scale coefficients in Chebyshev variable for the modified range <a>X</a> of variable i
  SCVar<T>& scale
    ( const unsigned i, const T&X );
  //! @brief Scale coefficients in Chebyshev variable for the modified variable ranges <a>X</a>
  SCVar<T>& scale
    ( const T*X );
  //! @brief Simplify the model by appending coefficient less than TOL to the remainder term
  SCVar<T>& simplify
    ( const double TOL=machprec() );
 /** @} */

  SCVar<T>& operator =
    ( const SCVar<T>& );
  SCVar<T>& operator =
    ( const double );
  SCVar<T>& operator =
    ( const T& );
  template <typename U> SCVar<T>& operator +=
    ( const SCVar<U>& );
  template <typename U> SCVar<T>& operator +=
    ( const U& );
  SCVar<T>& operator +=
    ( const double );
  template <typename U> SCVar<T>& operator -=
    ( const SCVar<U>& );
  template <typename U> SCVar<T>& operator -=
    ( const U& );
  SCVar<T>& operator -=
    ( const double );
  SCVar<T>& operator *=
    ( const SCVar<T>& );
  SCVar<T>& operator *=
    ( const double );
  SCVar<T>& operator *=
    ( const T& );
  SCVar<T>& operator /=
    ( const SCVar<T>& );
  SCVar<T>& operator /=
    ( const double );

private:
  //! @brief Polynomial range bounder using specified bounder <a>type</a> with basis functions <a>bndbasis</a> in U arithmetic
  template <typename U> U _polybound
    ( const U*const*bndbasis, const int type ) const
    { if( !_CM ) return !_coefmon.empty() && !_coefmon.begin()->first.first? _coefmon.begin()->second: 0.;
      return _CM->_polybound( _coefmon, bndbasis, type ); }

  //! @brief Polynomial range bounder using default bounder in U arithmetic
  template <typename U> U _polybound
    ( const U*const*bndbasis ) const
    { return _polybound( bndbasis, _CM? _CM->options.BOUNDER_TYPE: 0 ); }

  //! @brief Polynomial range bounder using specified bounder <a>type</a>
  T _polybound
    ( const int type ) const
    { return _polybound( (const T*const*)0, type ); }

  //! @brief Polynomial range bounder using default bounder
  T _polybound
    () const
    { return _polybound( _CM? _CM->options.BOUNDER_TYPE: 0 ); }

  //! @brief Prototype real-valued function for interpolation
  typedef double (puniv)
    ( const double x );

  //! @brief Construct Chebyshev interpolating polynomial coefficient <a>coefmon</a> for univariate <a>f</a>
  void _interpolation
    ( double*coefmon, puniv f ) const
    { SCModel<T>::_interpolation( coefmon, maxord()+_CM->options.INTERP_EXTRA, bound(), f ); }

  //! @brief Apply Chebyshev composition to variable <a>CVI</a> using the coefficients <a>coefmon</a> of the outer function
  template <typename U> SCVar<T> _composition
    ( const U* coefouter ) const
    { return SCModel<T>::_composition( coefouter, maxord(), *this ); }

  //! @brief Scale current variable in order for its range to be within [-1,1], with <a>c</a> and <a>w</a> respectively the center and width, respectively, of the orginal variable range
  SCVar<T> _rescale
    ( const double w, const double c ) const
    { return( !isequal(w,0.)? (*this-c)/w: c ); }

  //! @brief Scale object
  template <typename U> static U _rescale
    ( U&X, const double w, const double c )
    { return( !isequal(w,0.)? (X-c)/w: c ); }

  //! @brief Return an array of Chebyshev variables representing the coefficients in the univariate polynomial for variable <a>ivar</a> only
  SCVar<T>* _single
    ( const unsigned ivar );
};

////////////////////////////////// SCModel //////////////////////////////////////

template <typename T> template <typename U> inline U*
SCModel<T>::_get_bndpow
( const unsigned maxord, const U&bndvar, const double ref, const double scal )
{
  U *bndcheb = new U[maxord+1];
  bndcheb[0] = 1;
  if( maxord ) bndcheb[1] = ( bndvar - ref ) / scal;
  for( unsigned i=2; i<=maxord; i++ )
    bndcheb[i] = Op<U>::cheb( bndcheb[1], i );
 return bndcheb;
}

template <typename T> template <typename U> inline U*
SCModel<T>::_get_bndpow
( const unsigned maxord, const U&bndvar )
{
  U *bndcheb = new U[maxord+1];
  bndcheb[0] = 1;
  if( maxord ) bndcheb[1] = bndvar;
  for( unsigned i=2; i<=maxord; i++ )
    bndcheb[i] = Op<U>::cheb( bndcheb[1], i );
 return bndcheb;
}

template <typename T> template <typename U> inline U**
SCModel<T>::get_basis
( const unsigned maxord, const U*bndvar, const bool scaled ) const
{
  assert ( bndvar );
  U**bndcheb = new U*[_nvar];
  for( unsigned i=0; i<_nvar; i++ )
    bndcheb[i] = scaled? _get_bndpow( maxord, bndvar[i] ):
                         _get_bndpow( maxord, bndvar[i], _refvar[i], _scalvar[i] );
  return bndcheb;
}

template <typename T> template <typename U> inline void
SCModel<T>::get_bndmon
( std::map<t_expmon,U>&bndmon, const U*bndvar, const bool scaled ) const
{
  if( bndmon.empty() ) return;

  const unsigned maxord = bndmon.rbegin()->first.first;
  U**basis = get_basis( maxord, bndvar, scaled );

  auto it = bndmon.begin();
  if( !it->first.first ){ it->second = 1; ++it; }
  for( ; it!=bndmon.end(); ++it ){
    it->second = 1;
    for( auto ie=it->first.second.cbegin(); ie!=it->first.second.cend(); ++ie )
      it->second *= basis[ie->first][ie->second];
  }

  for( unsigned j=0; j<_nvar; j++) delete[] basis[j];
  delete[] basis;
}

template <typename T> inline void
SCModel<T>::_interpolation
( double*coefmon, const unsigned maxord, const T&X, puniv f )
{
  double b( Op<T>::mid(X) ), a( Op<T>::u(X)-b ), x[maxord+1], fx[maxord+1];
  double mulconst( PI/(2.*double(maxord+1)) );
  for( unsigned i(0); i<=maxord; i++ ){
    x[i]  = std::cos(mulconst*(2.*double(i)+1.));
    fx[i] = f( a*x[i]+b );
  }

  switch( maxord ){
  case 0:
    coefmon[0] = fx[0];
    return;
  case 1:
    coefmon[0] = 0.5 * ( fx[0] + fx[1] );
    coefmon[1] = ( fx[1] - fx[0] ) / ( x[1] - x[0] );
    return;
  default:
    for( unsigned i(0); i<=maxord; i++ ){
      double mulconst2( std::cos(mulconst*double(i)) ),
             mulconst3( 4*std::pow(mulconst2,2)-2 ), 
             b0( 0 ), b1( 0 );
      b0 = fx[maxord];
      b1 = fx[maxord-1] + mulconst3*b0;
      for( unsigned j=maxord-2; j>1; j-=2 ){
        b0 = fx[j] + mulconst3*b1 - b0;
        b1 = fx[j-1] + mulconst3*b0 - b1;
      }
      if( !(maxord%2) )
        b0 = fx[0] + mulconst3*b1 - b0 - b1;
      else{
        b0 = fx[1] + mulconst3*b1 - b0;
        b0 = fx[0] + mulconst3*b0 - b1 - b0;
      }
      coefmon[i] = 2./double(maxord+1)*mulconst2*b0;
    }
    coefmon[0] *=0.5;
    return;
  }
}

template <typename T> template <typename U> inline SCVar<T>
SCModel<T>::_composition
( const U* coefouter, const unsigned maxord, const SCVar<T>& CVinner )
{
  //composition based on http://en.wikipedia.org/wiki/Clenshaw_algorithm#Special_case_for_Chebyshev_series
  if( !maxord )
    return coefouter[0];

  else if( maxord == 1 )
    return CVinner * coefouter[1] + coefouter[0];

  SCVar<T> CVinnerx2 = 2. * CVinner;
//std::cout << "CVinner:" << CVinner;
//std::cout << "CVinnerx2:" << CVinnerx2;
  SCVar<T> CV1 = coefouter[maxord];
//std::cout << "CV1:" << CV1;
//for( unsigned i=0; i<CV1.nmon(); i++ ) std::cout << CV1._coefmon[i] << std::endl;
  SCVar<T> CV2 = coefouter[maxord-1] + CVinnerx2 * CV1;
//std::cout << "CV2:" << CV2;
//for( unsigned i=0; i<CV2.nmon(); i++ ) std::cout << CV2._coefmon[i] << std::endl;
  for( unsigned i=maxord-2; i>1; i-=2 ){
    CV1 = coefouter[i]   + CVinnerx2 * CV2 - CV1;
//std::cout << "CV1:" << CV1;
    CV2 = coefouter[i-1] + CVinnerx2 * CV1 - CV2;
//std::cout << "CV2:" << CV2;
  }
  if( !(maxord%2) )
    return coefouter[0] + CVinner * CV2 - CV1;
//SCVar<T> tmp = coefouter[1] + CVinnerx2 * CV2 - CV1;
//std::cout << "coefouter[1] + CVinnerx2 * CV2 - CV1:" << tmp;
//for( unsigned i=0; i<tmp.nmon(); i++ ) std::cout << tmp._coefmon[i] << std::endl;
  CV1 = coefouter[1] + CVinnerx2 * CV2 - CV1;
//for( unsigned i=0; i<CV1.nmon(); i++ ) std::cout << CV1._coefmon[i] << std::endl;
//std::cout << "CV1:" << CV1;
//std::cout << "CVinner * CV1:" << CVinner * CV1;
  return coefouter[0] + CVinner * CV1 - CV2;
}

template <typename T> inline SCVar<T>
SCModel<T>::_intpow
( const SCVar<T>&CV, const int n ) const
{
  if( n == 0 ) return 1.;
  else if( n == 1 ) return CV;
  return n%2 ? sqr( _intpow( CV, n/2 ) ) * CV : sqr( _intpow( CV, n/2 ) );
}

template <typename T> inline void
SCModel<T>::_sscal1D
( const t_coefmon& coefmon0, const double& dscal, t_coefmon& coefmon )
const
{
  if( isequal(dscal,0.) ) return;
  coefmon = coefmon0;
  if( isequal(dscal,1.) ) return;
  for( auto it=coefmon.begin(); it!=coefmon.end(); ++it )
    it->second *= dscal;
  return;
}

template <typename T> inline void
SCModel<T>::_slift1D
( const t_coefmon& coefmon0, const double& dscal, t_coefmon& coefmon )
const
{
  if( isequal(dscal,0.) ) return;
  for( auto it=coefmon0.begin(); it!=coefmon0.end(); ++it ){
    auto pmon = coefmon.insert( *it );
    if( pmon.second ){ if( !isequal(dscal,1.) ) pmon.first->second *= dscal; }
    else{ pmon.first->second += isequal(dscal,1.)? it->second: it->second * dscal; }
  }
}

template <typename T> inline void
SCModel<T>::_slift1D
( const t_coefmon& coefmon0, const double& dscal, t_coefmon& coefmon,
  double&coefrem, const unsigned ndxvar, const unsigned ndxord )
const
{
  t_expmon expmon;
  for( auto it=coefmon0.begin(); it!=coefmon0.end(); ++it, expmon.second.clear() ){
    if( it->first.first + ndxord > _maxord ){ // append to remainder coefficient if total order too large
      coefrem += std::fabs( isequal(dscal,1.)? it->second: it->second*dscal );
      continue;
    }
    expmon = it->first;
    expmon.first += ndxord;
    expmon.second.insert( expmon.second.begin(), std::make_pair(ndxvar,ndxord) );
    auto pmon = coefmon.insert( std::make_pair( expmon, isequal(dscal,1.)? it->second: it->second*dscal ) );
    if( !pmon.second ) pmon.first->second += isequal(dscal,1.)? it->second: it->second*dscal;
  }
}

template <typename T> inline void
SCModel<T>::_slift1D
( const t_coefmon& coefmon0, double&coefrem )
const
{
  for( auto it=coefmon0.begin(); it!=coefmon0.end(); ++it )
    coefrem += std::fabs( it->second );
}

template <typename T> inline void
SCModel<T>::_sdisp1D
( const t_coefmon& coefmon, const std::string&name, std::ostream&os )
const
{
  os << name;
  for( auto it=coefmon.begin(); it!=coefmon.end(); ++it ){
    if( it != coefmon.begin() ) os << " + ";
    os << it->second;
    for( auto ie=it->first.second.begin(); ie!=it->first.second.end(); ++ie )
      os << "·T" << ie->second << "[" << ie->first << "]";
  }
  os << std::endl;
}

template <typename T> inline void
SCModel<T>::_sdisp1D
( const std::vector<t_coefmon>& coefvec, const unsigned ndxvar, 
  const std::string&name, std::ostream&os )
const
{
  os << name;
  for( unsigned i=0; i<=_maxord; i++ ){
    if( i ) os << " + T" << i << "[" << ndxvar << "] ·";
    os << " { ";
    for( auto it=coefvec[i].begin(); it!=coefvec[i].end(); ++it ){
      if( it != coefvec[i].begin() ) os << " + ";
      os << it->second;
      for( auto ie=it->first.second.begin(); ie!=it->first.second.end(); ++ie )
        os << "·T" << ie->second << "[" << ie->first << "]";
    }
    os << " }";
  }
  os << std::endl;
}

template <typename T> inline void
SCModel<T>::_sdisp1D
( const std::vector<SCVar<T>>& vec, const unsigned ndxvar, 
  const std::string&name, std::ostream&os )
const
{
  os << name;
  for( unsigned i=0; i<=_maxord; i++ ){
    if( i ) os << " + T" << i << "[" << ndxvar << "] ·";
    os << " { ";
    for( auto it=vec[i].coefmon().begin(); it!=vec[i].coefmon().end(); ++it ){
      if( it != vec[i].coefmon().begin() ) os << " + ";
      os << it->second;
      for( auto ie=it->first.second.begin(); ie!=it->first.second.end(); ++ie )
        os << "·T" << ie->second << "[" << ie->first << "]";
    }
    os << " }";
  }
  os << std::endl;
}

template <typename T> inline void
SCModel<T>::_sprod1D
( const std::vector<t_coefmon>& CV1vec,
  const std::vector<t_coefmon>& CV2vec,
  t_coefmon& coefmon, double&coefrem,
  const unsigned ndxvar )
const
{
  // construct product matrix of polynomial coefficients
  std::vector<t_coefmon> CVprod( (_maxord+1)*(_maxord+1) );
  for( unsigned iord1=0; iord1<=_maxord; iord1++ ){
    if( CV1vec[iord1].empty() ) continue; // no terms
    if( CV1vec[iord1].size() == 1 && !CV1vec[iord1].begin()->first.first ){ // constant term only
      for( unsigned iord2=0; iord2<=_maxord; iord2++ )
        _sscal1D( CV2vec[iord2], CV1vec[iord1].begin()->second, CVprod[iord1*(_maxord+1)+iord2] );
      continue;
    }
    for( unsigned iord2=0; iord2<=_maxord; iord2++ ){ // general polynomial
      if( CV2vec[iord2].empty() ) continue; // no term
      if( CV2vec[iord2].size() == 1 && !CV2vec[iord2].begin()->first.first ){ // constant term only
        _sscal1D( CV1vec[iord1], CV2vec[iord2].begin()->second, CVprod[iord1*(_maxord+1)+iord2] );
        continue;
      }
#ifdef MC__POLYMODEL_DEBUG_SPROD
      std::cout << "Term (" << iord1 << "," << iord2 << "):\n";
#endif
      std::vector<t_coefmon> CV11vec( _maxord+1 ), CV22vec( _maxord+1 ); // component vectors for both coefficients
      for( auto it=CV1vec[iord1].begin(); it!=CV1vec[iord1].end(); ++it )
        _svec1D( ndxvar+1, it, CV11vec );
#ifdef MC__POLYMODEL_DEBUG_SPROD
      _sdisp1D( CV11vec, ndxvar+1, "Var #1: " );
#endif
      for( auto it=CV2vec[iord2].begin(); it!=CV2vec[iord2].end(); ++it )
        _svec1D( ndxvar+1, it, CV22vec );
#ifdef MC__POLYMODEL_DEBUG_SPROD
      _sdisp1D( CV22vec, ndxvar+1, "Var #2: " );
#endif
      _sprod1D( CV11vec, CV22vec, CVprod[iord1*(_maxord+1)+iord2], coefrem, ndxvar+1 );
    }
  }

  // construct 1D product result and augment remainder as appropriate
  coefmon.clear();
  for( unsigned l=0; l<=_maxord; l++ )
    _slift1D( CVprod[l*(_maxord+1)+l], l?1.:2., coefmon );
  for( unsigned k=1; k<=_maxord; k++ ){
    for( unsigned l=0; l<=k; l++ )
      _slift1D( CVprod[(k-l)*(_maxord+1)+l], l&&k-l?1.:2., coefmon, coefrem, ndxvar, k );
    for( unsigned l=1; l+k<=_maxord; l++ ){
      _slift1D( CVprod[l*(_maxord+1)+l+k],   1., coefmon, coefrem, ndxvar, k );
      _slift1D( CVprod[(l+k)*(_maxord+1)+l], 1., coefmon, coefrem, ndxvar, k );
    }
  }
  for( unsigned k=_maxord+1; k<=2*_maxord; k++ )
    for( unsigned l=k-_maxord; l<=_maxord; l++ )
      _slift1D( CVprod[(k-l)*(_maxord+1)+l], coefrem );
  // rescale the coefficients by 0.5
  for( auto it=coefmon.begin(); it!=coefmon.end(); ++it )
    it->second *= 0.5;
#ifdef MC__POLYMODEL_DEBUG_SPROD
  _sdisp1D( coefmon, "Prod: " );
#endif
}

template <typename T> inline void
SCModel<T>::_svec1D
( const unsigned ndxvar, typename t_coefmon::const_iterator it,
  std::vector<t_coefmon>& vec )
const
{
  auto ie = it->first.second.begin();
  if( !it->first.first || ie->first != ndxvar ) // no dependence on variable #ndxvar 
    vec[ 0 ].insert( *it );
  else      // dependence on variable #ndxvar of order ie
    vec[ ie->second ].insert( std::make_pair( std::make_pair( it->first.first-ie->second,
      std::map<unsigned,unsigned>( ++it->first.second.begin(),it->first.second.end() ) ),
      it->second ) );
}

template <typename T> inline void
SCModel<T>::_svec1Dfull
( const unsigned ndxvar, typename t_coefmon::const_iterator it,
  std::vector<SCVar<T>>& vec )
const
{
  auto ie = it->first.second.find( ndxvar );
  if( ie == it->first.second.end() ) // no dependence on variable #ndxvar 
    vec[ 0 ].coefmon().insert( *it );
  else{
    t_expmon ex( it->first.first-ie->second, it->first.second );
    ex.second.erase( ndxvar ); // remove T[ndx] entry
    vec[ ie->second ].coefmon().insert( std::make_pair( ex, it->second ) );
  }
}

template <typename T> inline void
SCModel<T>::_sprod
( const SCVar<T>&CV1, const SCVar<T>&CV2, t_coefmon& coefmon,
  double&coefrem )
const
{
  // Construct vectors of coefficients for variable #0
  std::vector<t_coefmon> CV1vec( _maxord+1 ), CV2vec( _maxord+1 );
  for( auto it=CV1._coefmon.begin(); it!=CV1._coefmon.end(); ++it )
    _svec1D( 0, it, CV1vec );
#ifdef MC__POLYMODEL_DEBUG_SPROD
  _sdisp1D( CV1vec, 0, "Var #1: " );
#endif
  for( auto it=CV2._coefmon.begin(); it!=CV2._coefmon.end(); ++it )
    _svec1D( 0, it, CV2vec );
#ifdef MC__POLYMODEL_DEBUG_SPROD
  _sdisp1D( CV2vec, 0, "Var #2: " );
#endif

  // Call recursive product of univariate Chebyshev polynomials
  _sprod1D( CV1vec, CV2vec, coefmon, coefrem, 0 );
  coefrem *= 0.5;
}

template <typename T> inline void
SCModel<T>::_ssqr
( const SCVar<T>&CV, t_coefmon&coefmon, double&coefrem )
const
{
  // Construct vectors of coefficients for variable #0
  std::vector<t_coefmon> CVvec( _maxord+1 );
  for( auto it=CV._coefmon.begin(); it!=CV._coefmon.end(); ++it )
    _svec1D( 0, it, CVvec );
#ifdef MC__POLYMODEL_DEBUG_SSQR
  _sdisp1D( CVvec, 0, "Var: " );
#endif

  // Call recursive product of univariate Chebyshev polynomials
  _sprod1D( CVvec, CVvec, coefmon, coefrem, 0 );
  coefrem *= 0.5;
}

// ==> account for sparse format

template <typename T> template <typename C, typename U> inline U
SCModel<T>::_polybound_eigen
( const std::map< t_expmon, C >& coefmon, const U*const*bndbasis ) const
{
  return _polybound_naive( coefmon, bndbasis );
/*
  if( _maxord < 2 ) return _polybound_naive( coefmon, bndbasis );

  static const double TOL = 1e-8;

  U bndpol = (ndxmon.empty() || ndxmon.find(0)!=ndxmon.end())? coefmon[0]: 0.;
  //U bndpol = coefmon[0];
  if( _maxord == 1 ) bndpol += bndord[1];

  else if( _maxord > 1 ){
    double*Umat = new double[_nvar*_nvar];
    for( unsigned i=_posord[2]; i<_posord[3]; i++ ){
      unsigned i1=0, i2=_nvar;
      C Ci = (ndxmon.empty() || ndxmon.find(i)!=ndxmon.end())? coefmon[i]: 0.;
      const unsigned*iexp=_expmon+i*_nvar;
      for( ; i1<_nvar; i1++ ) if( iexp[i1] ) break;
      if( iexp[i1] == 2 ){
        Umat[_nvar*i1+i1] = 2.*Ci;
        bndpol -= Ci;
        continue;
      }
      for( i2=i1+1; i2<_nvar; i2++ ) if( iexp[i2] ) break;
      Umat[_nvar*i1+i2] = 0.;
      Umat[_nvar*i2+i1] = Ci/2.;
    }
#ifdef MC__POLYMODEL_DEBUG_POLYBOUND
    display( _nvar, _nvar, Umat, _nvar, "Matrix U", std::cout );
#endif
    double*Dmat = mc::dsyev_wrapper( _nvar, Umat, true );
    if( !Dmat ){
      delete[] Umat;
      return _polybound_LSB( coefmon, bndord, bndbasis );
    }

    for( unsigned i=0; i<_nvar; i++ ){
      double linaux = 0.;
      U bndaux(0.);
      for( unsigned k=0; k<_nvar; k++ ){
        C Ck = (ndxmon.empty() || ndxmon.find(_nvar-k)!=ndxmon.end())? coefmon[_nvar-k]: 0.;
        linaux += Umat[i*_nvar+k] * Ck;
        bndaux += Umat[i*_nvar+k] * bndbasis[k][1];
      }
#ifdef MC__POLYMODEL_DEBUG_POLYBOUND
      std::cout << i << ": LINAUX = " << linaux
                << "  BNDAUX = " << bndaux << std::endl;
#endif
      if( std::fabs(Dmat[i]) > TOL )
        bndpol += Dmat[i] * Op<U>::sqr( linaux/Dmat[i]/2. + bndaux )
                - linaux*linaux/Dmat[i]/4.;
      else     
        bndpol += linaux * bndaux + Dmat[i] * Op<U>::sqr( bndaux );
#ifdef MC__POLYMODEL_DEBUG_POLYBOUND
        std::cout << "BNDPOL: " << bndpol << std::endl;
#endif
    }
    delete[] Umat;
    delete[] Dmat;
  }
#ifdef MC__POLYMODEL_DEBUG_POLYBOUND
  int tmp; std::cin >> tmp;
#endif

  for( unsigned i=3; i<=_maxord; i++ ) bndpol += bndord[i];
  return bndpol;
*/
}

// ==> account for sparse format

template <typename T> template <typename C, typename U> inline U
SCModel<T>::_polybound_LSB
( const std::map< t_expmon, C >& coefmon, const U*const*bndbasis ) const
{
  // Constant or linear model
  if( coefmon.empty() || coefmon.rbegin()->first.first < 2 )
    return _polybound_naive( coefmon, bndbasis );

  // Quadratic terms in combination with linear terms
  static const double TOL = 1e-8;
  U bndpol = coefmon.cbegin()->first.first? 0.: coefmon.cbegin()->second;
  auto it1 = coefmon.lower_bound( std::make_pair( 1, std::map<unsigned,unsigned>() ) );
  auto it2 = coefmon.lower_bound( std::make_pair( 2, std::map<unsigned,unsigned>() ) );
  auto it3 = coefmon.lower_bound( std::make_pair( 3, std::map<unsigned,unsigned>() ) );
  std::map< t_expmon, C > coeflin; coeflin.insert( it1, it2 );
  for( ; it2!=it3; ++it2 ){
    auto ie2 = it2->first.second.begin();
    if( ie2->second == 1 ){ // off-diagonal quadratic terms
      bndpol += it2->second * ( bndbasis? bndbasis[ie2->first][1] * bndbasis[(++ie2)->first][1]
                                        : 2.*Op<T>::zeroone()-1. );
      continue;
    }
    t_expmon explin( 1, std::map<unsigned,unsigned>() );
    explin.second.insert( std::make_pair( ie2->first, 1 ) ); 
    it1 = coeflin.find( explin );
    if( it1 != coeflin.end() && std::fabs(it2->second) > TOL ){
       bndpol += (2.*it2->second) * Op<U>::sqr( (bndbasis? bndbasis[ie2->first][1]: 2.*Op<T>::zeroone()-1)
                 + it1->second/(it2->second*4.) ) - it2->second - it1->second*it1->second/8./it2->second;
       coeflin.erase( it1 );
    }
    else if( it1 != coeflin.end() ){
      bndpol += it2->second * ( bndbasis? bndbasis[ie2->first][2]: 2.*Op<T>::zeroone()-1. )
              + it1->second * ( bndbasis? bndbasis[ie2->first][1]: 2.*Op<T>::zeroone()-1. );
       coeflin.erase( it1 );
    }
    else
      bndpol += it2->second * ( bndbasis? bndbasis[ie2->first][2]: 2.*Op<T>::zeroone()-1. );
  }

  // Remaining linear terms
  for( it1=coeflin.begin(); it1!=coeflin.end(); ++it1 ){
    auto ie1 = it1->first.second.begin();
    bndpol += it1->second * ( bndbasis? bndbasis[ie1->first][1]: 2.*Op<T>::zeroone()-1. );
  }

  // Thrid and higher-order terms
  bndpol += _polybound_naive( coefmon, bndbasis, 3 );
  return bndpol;
}
/*
template <typename T> template <typename C, typename U> inline U
SCModel<T>::_polybound_bernstein
( const C*coefmon, const U*bndord, const U*const*bndbasis,
  const std::set<unsigned>&ndxmon )
{
  // Expand binomial coefficient and exponent arrays if needed
#ifdef MC__SCVAR_DEBUG_BERNSTEIN
  std::cout << "binom max: " << _binom_size.first << "  "
            << _binom_size.second << std::endl;
#endif
  const unsigned maxord = (options.BOUNDER_ORDER>_maxord? 
    options.BOUNDER_ORDER: _maxord );
  const poly_size maxmon = std::pow(maxord+1,_nvar);
  _ext_expmon( maxord, true );
  _ext_binom( 2*_maxord );
#ifdef MC__SCVAR_DEBUG_BERNSTEIN
  std::cout << "binom max: " << _binom_size.first << "  "
            << _binom_size.second << std::endl;
#endif

  // Compute transformation matrix
  double*trmat = new double[(maxord+1)*(_maxord+1)];
#ifdef MC__SCVAR_DEBUG_BERNSTEIN
  std::cout << "trmat:\n" << std::scientific << std::setprecision(5);
#endif
  for( unsigned j=0, jk=0; j<=maxord; j++ ){
    for( unsigned k=0; k<=_maxord; k++, jk++ ){
      trmat[jk] = _transform_bernstein( j, k, maxord );
#ifdef MC__SCVAR_DEBUG_BERNSTEIN
      std::cout << "  " << trmat[jk];
#endif
    }
#ifdef MC__SCVAR_DEBUG_BERNSTEIN
    std::cout << std::endl;
#endif
  }

  // Compute min/max amongst all Bernstein coefficients
  U bndpol = (ndxmon.empty() || ndxmon.find(0)!=ndxmon.end())? coefmon[0]: 0.;
  //U bndpol = coefmon[0];
#ifdef MC__SCVAR_DEBUG_BERNSTEIN
  std::cout << "\n0:  " << bndpol << std::endl;
#endif
  for( poly_size jmon=0; jmon<maxmon; jmon++ ){ // Loop over Bernstein terms
    const unsigned*jexp = _expmon + jmon*_nvar;
    C coefbern = _coef_bernstein( coefmon, jexp, trmat, ndxmon );
    bndpol = Op<U>::hull( bndpol, coefbern );
#ifdef MC__SCVAR_DEBUG_BERNSTEIN
    std::cout << jmon << " ["; 
    for( unsigned ivar=0; ivar<_nvar; ivar++ )
      std::cout << std::setw(3) << jexp[ivar];
    std::cout << "] : " << coefbern << " : " << bndpol << std::endl;
#endif
  }

  delete[] trmat;
  return bndpol;
}

template <typename T> inline double
SCModel<T>::_transform_bernstein
( const unsigned j, const unsigned k, const unsigned n ) const
{
  double Mjk = 0.;
  for( unsigned i=(j+k<=n?0:j+k-n); i<=(j<=k?j:k); i++ ){
    double Mjki = (double)_get_binom(2*k,2*i) * (double)_get_binom(n-k,j-i);
    (k+i)%2? Mjk -= Mjki: Mjk += Mjki;
    //std::cout << k << "  " << i << " (k-i)%2: " << (k-i)%2 << std::endl;
  }
  Mjk /= (double)_get_binom(n,j);
  return Mjk;
}

template <typename T> template <typename C> inline C
SCModel<T>::_coef_bernstein
( const std::map< t_expmon, C >& coefmon, const unsigned*jexp,
  const double*trmat ) const
{
  // Compute bernstein coefficient with variables indices <tt>jexp</tt>
  C coefbern = (ndxmon.empty() || ndxmon.find(0)!=ndxmon.end())? coefmon[0]: 0.;
#ifdef MC__SCVAR_DEBUG_BERNSTEIN
    std::cout << "  [  0]  " << coefmon[0] << "  " << coefbern << std::endl;
#endif
  // Sparse loop over Chebyshev terms
  for( auto it=ndxmon.begin(); it!=ndxmon.end(); ++it ){
    const unsigned*kexp = _expmon + (*it)*_nvar;
    C termbern = coefmon[(*it)]; // Chebyshev coefficient for (*it)
#ifdef MC__SCVAR_DEBUG_BERNSTEIN
    std::cout << "  ["; 
    for( unsigned ivar=0; ivar<_nvar; ivar++ )
      std::cout << std::setw(3) << kexp[ivar];
    std::cout << "]";
#endif
    for( unsigned ivar=0; ivar<_nvar; ivar++ )
      termbern *= trmat[jexp[ivar]*(_maxord+1)+kexp[ivar]];
    coefbern += termbern;      
#ifdef MC__SCVAR_DEBUG_BERNSTEIN
    std::cout << "  " << termbern << "  " << coefbern << std::endl;
#endif
  }  
  // Dense loop over Chebyshev terms
  for( unsigned kmon=1; ndxmon.empty() && kmon<_nmon; kmon++ ){
    const unsigned*kexp = _expmon + kmon*_nvar;
    C termbern = coefmon[kmon]; // Chebyshev coefficient for kmon
#ifdef MC__SCVAR_DEBUG_BERNSTEIN
    std::cout << "  ["; 
    for( unsigned ivar=0; ivar<_nvar; ivar++ )
      std::cout << std::setw(3) << kexp[ivar];
    std::cout << "]";
#endif
    for( unsigned ivar=0; ivar<_nvar; ivar++ )
      termbern *= trmat[jexp[ivar]*(_maxord+1)+kexp[ivar]];
    coefbern += termbern;      
#ifdef MC__SCVAR_DEBUG_BERNSTEIN
    std::cout << "  " << termbern << "  " << coefbern << std::endl;
#endif
  }
#ifdef MC__SCVAR_DEBUG_BERNSTEIN2
  std::cout << std::endl;
#endif
  return coefbern;
}
*/
template <typename T> template <typename C, typename U> inline U
SCModel<T>::_polybound_naive
( const std::map< t_expmon, C >& coefmon, const U*const*bndbasis,
  const unsigned minord ) const
{
  // Empty model
  if( coefmon.empty() || coefmon.rbegin()->first.first < minord ) return 0.;
  auto it = coefmon.lower_bound( std::make_pair(minord,std::map<unsigned,unsigned>()) );

  // Polynomial bounding in T arithmetic
  if( !bndbasis ){
    double bndcst = 0., bndcoef = 0.;
    if( !it->first.first ){ bndcst = it->second; ++it; }
    for( ; it!=coefmon.end(); ++it )
      bndcoef += std::fabs( it->second );
    return bndcoef * (2.*Op<T>::zeroone()-1.) + bndcst;
  }

  // Polynomial bounding in U arithmetic
  U bndpol( 0. );
  if( !it->first.first ){ bndpol = it->second; ++it; }
  for( ; it!=coefmon.end(); ++it ){
    U bndmon( 1. );
    for( auto ie=it->first.second.begin(); ie!=it->first.second.end(); ++ie )
      bndmon *= bndbasis[ie->first][ie->second];
    bndpol += it->second  * bndmon;
  }
  return bndpol;
}

template <typename T> template <typename C, typename U> inline U
SCModel<T>::_polybound
( const std::map< t_expmon, C >& coefmon, const U*const*bndbasis,
  const int type )
{
  switch( type ){
  //case Options::BERNSTEIN:
  // return _polybound_bernstein( coefmon, bndbasis );
  case Options::EIGEN:
    return _polybound_eigen( coefmon, bndbasis );
  case Options::LSB:
    return _polybound_LSB( coefmon, bndbasis );
  case Options::NAIVE: default:
    return _polybound_naive( coefmon, bndbasis );
  }
}

////////////////////////////////// SCVar ///////////////////////////////////////

template <typename T> inline SCVar<T>&
SCVar<T>::operator =
( const SCVar<T>&var )
{
  _CM = var._CM;
  _set( var );
  return *this;
}

template <typename T> inline
SCVar<T>::SCVar
( SCModel<T>*CM )
: SPolyVar<T>(), _CM( CM )
{
  _bndrem = 0.;
  _set_bndpol( 0. );
}

template <typename T> inline
SCVar<T>::SCVar
( const double d, SCModel<T>*CM )
: SPolyVar<T>(), _CM( CM )
{
  if( isequal( d, 0. ) ) return;
  _coefmon.insert( std::make_pair( std::make_pair(0,std::map<unsigned,unsigned>()), d ) );
  _bndrem = 0.;
  _set_bndpol( d );
}

template <typename T> inline SCVar<T>&
SCVar<T>::operator =
( const double d )
{
  _reinit();
  _CM = 0;
  _coefmon.insert( std::make_pair( std::make_pair(0,std::map<unsigned,unsigned>()), d ) );
  _bndrem = 0.;
  _set_bndpol( d );
  _unset_bndT();
  return *this;
}

template <typename T> inline
SCVar<T>::SCVar
( const T&B, SCModel<T>*CM )
: SPolyVar<T>(), _CM( CM )
{
  const double midB = Op<T>::mid(B);
  _coefmon.insert( std::make_pair( std::make_pair(0,std::map<unsigned,unsigned>()), midB ) );
  _bndrem = B - midB;
  _set_bndpol( midB );
}

template <typename T> inline SCVar<T>&
SCVar<T>::operator =
( const T&B )
{
  _reinit();
  _CM = 0;
  const double midB = Op<T>::mid(B);
  _coefmon.insert( std::make_pair( std::make_pair(0,std::map<unsigned,unsigned>()), midB ) );
  _bndrem = B - midB;
  _set_bndpol( midB );
  _unset_bndT();
  return *this;
}

/*
// ==> account for sparse format

template <typename T> template <typename U> inline
SCVar<T>::SCVar
( SCModel<T>*&CM, const SCVar<U>&CV )
: SPolyVar<T>( CM ), _CM( CM )
{
  SCVar<U> CVtrunc( CV );
  _coefmon[0] = CVtrunc._coefmon[0];
  CVtrunc._coefmon[0] = 0. ;
  for( unsigned i=1; CM && i<nmon(); i++ ){
    if( CVtrunc.CM && i < CVtrunc.nmon() ){
      _coefmon[i] = CVtrunc._coefmon[i];
      CVtrunc._coefmon[i] = 0.;
    }
    else
      _coefmon[i] = 0.;
  }
  CVtrunc._bndord_uptd = false;
  CVtrunc._unset_bndpol();
  *_bndrem = T( CVtrunc.B() );
  _bndord_uptd = false;
  //_unset_bndpol();
  //_unset_bndT();
  if( CM && CM->options.MIXED_IA ) _set_bndT( CV._bndT );
  return;
}

// ==> account for sparse format

template <typename T> template <typename U> inline
SCVar<T>::SCVar
( SCModel<T>*&CM, const SCVar<U>&CV, T (*method)( const U& ) )
: SPolyVar<T>( CM ), _CM( CM )
{
  if( !method ) throw typename SCModel<T>::Exceptions( SCModel<T>::Exceptions::INIT );
  SCVar<U> CVtrunc( CV );
  _coefmon[0] = CVtrunc._coefmon[0];
  CVtrunc._coefmon[0] = 0. ;
  for( unsigned i=1; CM && i<nmon(); i++ ){
    if( CVtrunc.CM && i < CVtrunc.nmon() ){
      _coefmon[i] = CVtrunc._coefmon[i];
      CVtrunc._coefmon[i] = 0.;
    }
    else
      _coefmon[i] = 0.;
  }
  CVtrunc._bndord_uptd = false;
  CVtrunc._unset_bndpol();
  *_bndrem = (*method)( CVtrunc.B() );
  _bndord_uptd = false;
  //_unset_bndpol();
  //_unset_bndT();
  if( CM && CM->options.MIXED_IA ) _set_bndT( CV._bndT? (*method)( *CV._bndT ): 0 );
  return;
}

// ==> account for sparse format

template <typename T> template <typename U> inline
SCVar<T>::SCVar
( SCModel<T>*&CM, const SCVar<U>&CV, const T& (U::*method)() const )
: SPolyVar<T>( CM ), _CM( CM )
{
  if( !method ) throw typename SCModel<T>::Exceptions( SCModel<T>::Exceptions::INIT );
  SCVar<U> CVtrunc( CV );
  _coefmon[0] = CVtrunc._coefmon[0];
  CVtrunc._coefmon[0] = 0. ;
  for( unsigned i=1; CM && i<nmon(); i++ ){
    if( CVtrunc.CM && i < CVtrunc.nmon() ){
      _coefmon[i] = CVtrunc._coefmon[i];
      CVtrunc._coefmon[i] = 0.;
    }
    else
      _coefmon[i] = 0.;
  }
  CVtrunc._bndord_uptd = false;
  CVtrunc._unset_bndpol();
  *_bndrem = (CVtrunc.B().*method)();
  _bndord_uptd = false;
  //_unset_bndpol();
  //_unset_bndT();
  if( CM && CM->options.MIXED_IA ) _set_bndT( CV._bndT? (CV._bndT->*method)(): 0 );
  return;
}
*/

template <typename T> inline
SCVar<T>::SCVar
( SCModel<T>*CM, const unsigned ivar, const T&X )
: _CM( CM )
{
  _set( ivar, X );
}

template <typename T> inline SCVar<T>&
SCVar<T>::_set
( const unsigned i, const T&X, const bool updMod )
{
  if( !_CM ) throw typename SCModel<T>::Exceptions( SCModel<T>::Exceptions::INIT );  

  // Keep data for variable #ivar in model environment
  if( updMod ) _CM->_set( i, X );

  // Populate model variable
  _coefmon.clear();
  if( !isequal(_scalvar(i),0.) )
    _coefmon.insert( std::make_pair( std::make_pair(0,std::map<unsigned,unsigned>()),_refvar(i) ) );
  if( _CM->_maxord && !isequal(_scalvar(i),0.) ){
    std::map<unsigned,unsigned> ndx_i; ndx_i.insert( std::make_pair(i,1) );
    _coefmon.insert( std::make_pair( std::make_pair(1,ndx_i),_scalvar(i) ) );
    _set_bndpol( _bndvar(i) );
    _bndrem = 0.;
  }
  else{
    _set_bndpol( _refvar(i) );
    _bndrem = _bndvar(i) - _refvar(i);
  }
  //std::cout << "coefmon size: " << _coefmon.size() << std::endl;

  // Interval bounds
  //_unset_bndT();
  if( _CM->options.MIXED_IA ) _set_bndT( _bndvar(i) );

  return *this;
}

template <typename T> inline double
SCVar<T>::polynomial
( const double*x ) const
{
  // -> Is there a multivariate version of Clenshaw? Use recursively?

  double Pval = 0.;
  // Case: sparse storage
  for( auto it=_coefmon.begin(); it!=_coefmon.end(); ++it ){
    double valmon = it->second;
    for( auto ie=it->first.second.begin(); ie!=it->first.second.end(); ++ie )
      valmon *= _scalvar(ie->first)!=0.?
                mc::cheb((x[ie->first]-_refvar(ie->first))/_scalvar(ie->first),ie->second):
                _refvar(ie->first);
    Pval += valmon;
    //std::cout << Pval << " += " << it->second << " x " << valmon << std::endl;
  }
  return Pval;
}

template <typename T> inline double
SCVar<T>::constant( const bool reset )
{
  auto it_0 = ( _coefmon.empty() || _coefmon.begin()->first.first? _coefmon.end(): _coefmon.begin() );
  const double coefcst = ( it_0 == _coefmon.end()? 0.: it_0->second );
  if( reset && it_0 != _coefmon.end() ){
    _coefmon.erase( it_0 );
    if( _bndpol ) *_bndpol -= coefcst;
    if( _bndT )   *_bndT -= coefcst;
  }
  return coefcst;
}

template <typename T> inline double
SCVar<T>::linear
( const unsigned i, const bool reset )
{
  if( i>=nvar() || !maxord() ) return 0.;
  std::map<unsigned,unsigned> ndx_i; ndx_i.insert( std::make_pair(i,1) );
  auto it_i = ( _coefmon.empty()? _coefmon.end(): _coefmon.find( std::make_pair(1,ndx_i) ) );
  const double coeflin = ( it_i == _coefmon.end() || isequal(_scalvar(i),0.)? 0.: it_i->second/_scalvar(i) );
  if( reset && it_i != _coefmon.end() ){
    _coefmon.erase( it_i );
    _unset_bndpol();
    _unset_bndT();
  }
  return coeflin;
}
/*
template <typename T> inline SCVar<T>
SCVar<T>::scale
( const T*X ) const
{
  SCVar<T> CVscal( *this );
  // Return *this if null pointer to model _CM or variable ranges X
  for( unsigned i=0; _CM && X && i<nvar(); i++ ){
    // Nothing to scale if range of current variable #i did not change
    if( isequal( Op<T>::l(X[i]), Op<T>::l(_bndvar(i)) )
     && isequal( Op<T>::u(X[i]), Op<T>::u(_bndvar(i)) ) ) continue;

    // Get coefficients in univariate polynomial representation w.r.t variable #i
    std::vector<SCVar<T>> CVcoefi( maxord()+1 );
    for( auto it=CVscal._coefmon.begin(); it!=CVscal._coefmon.end(); ++it )
      _CM->_svec1D( i, it, CVcoefi );
#ifdef MC__POLYMODEL_DEBUG_SCALE
    _CM->_sdisp1D( CVcoefi, i, "Var #i: " );
#endif

    // Nothing to scale if independent of current variable #i
    bool nodep = true;
    for( unsigned k=1; nodep && k<=maxord(); k++ )
      if( !CVcoefi[k].coefmon().empty() ) nodep = false;
    if( nodep ) continue;

    // Compose with rescaled inner variable
    SCVar<T> CVi( _CM ); CVi._set( i, X[i], false );
#ifdef MC__POLYMODEL_DEBUG_SCALE
    std::cout << "CVi[" << i << "]:" << CVi;
#endif
    if( !isequal(_scalvar(i),0.) ){
      CVi.constant( true );
      CVi *= Op<T>::diam(X[i]) / (2.*_scalvar(i) );
      CVi += Op<T>::mid(X[i]);
    }
#ifdef MC__POLYMODEL_DEBUG_SCALE
    std::cout << "CVi[" << i << "]:" << CVi;
#endif
    CVi = CVi._rescale( _scalvar(i), _refvar(i) );
#ifdef MC__POLYMODEL_DEBUG_SCALE
    std::cout << "CVi[" << i << "]:" << CVi;
#endif
    CVscal = CVi._composition( CVcoefi.data() );
    CVscal += _bndrem;
    CVscal._unset_bndpol();
    CVscal._unset_bndT();
  }
  return CVscal;
}
*/
template <typename T> inline SCVar<T>&
SCVar<T>::scale
( const unsigned i, const T&X )
{
  // Nothing to do if model _CM is NULL, i is outside of variable range,
  // or variable range X did not change
  if( !_CM || i >= nvar()
   || ( isequal( Op<T>::l(X), Op<T>::l(_bndvar(i)) )
     && isequal( Op<T>::u(X), Op<T>::u(_bndvar(i)) ) ) ) return *this;

  // Get coefficients in univariate polynomial representation w.r.t variable #i
  std::vector<SCVar<T>> CVcoefi( maxord()+1 );
  for( auto it=_coefmon.begin(); it!=_coefmon.end(); ++it )
    _CM->_svec1Dfull( i, it, CVcoefi );
#ifdef MC__POLYMODEL_DEBUG_SCALE
  _CM->_sdisp1D( CVcoefi, i, "Var #i: " );
#endif

  // Nothing to scale if independent of current variable #i
  bool nodep = true;
  for( unsigned k=1; nodep && k<=maxord(); k++ )
    if( !CVcoefi[k].coefmon().empty() ) nodep = false;
  if( nodep ) return *this;

  // Compose with rescaled inner variable
  SCVar<T> CVi( _CM ); CVi._set( i, X, false );
#ifdef MC__POLYMODEL_DEBUG_SCALE
  std::cout << "CVi[" << i << "]:" << CVi;
#endif
  if( !isequal(_scalvar(i),0.) ){
    CVi -= _refvar(i); //CVi.constant( true );
    CVi *= Op<T>::diam(X) / (2.*_scalvar(i) );
    CVi += Op<T>::mid(X);
  }
#ifdef MC__POLYMODEL_DEBUG_SCALE
  std::cout << "CVi[" << i << "]:" << CVi;
#endif
  CVi = CVi._rescale( _scalvar(i), _refvar(i) );
#ifdef MC__POLYMODEL_DEBUG_SCALE
  std::cout << "CVi[" << i << "]:" << CVi;
#endif
/*
  // Compose with rescaled inner variable
  const double scalvar0 = _scalvar(i), refvar0 = _refvar(i);
  SCVar<T> CVi( _CM ); CVi._set( i, X );
#ifdef MC__POLYMODEL_DEBUG_SCALE
  std::cout << "CVi[" << i << "]:" << CVi;
#endif
  CVi = CVi._rescale( scalvar0, refvar0 );
#ifdef MC__POLYMODEL_DEBUG_SCALE
  std::cout << "CVi[" << i << "]:" << CVi;
#endif
*/
  _coefmon = CVi._composition( CVcoefi.data() ).coefmon();
  _unset_bndpol();
  _unset_bndT();
  return *this;
}

template <typename T> inline SCVar<T>&
SCVar<T>::scale
( const T*X )
{
  // Return *this if null pointer to model _CM or variable ranges X
  for( unsigned i=0; _CM && X && i<nvar(); i++ )
    scale( i, X[i] );
  return *this;
}

template <typename T> inline SCVar<T>&
SCVar<T>::simplify
( const double TOL )
{
  // Sparse multivariate polynomial
  for( auto it=_coefmon.begin(); it!=_coefmon.end(); ){
    if( std::fabs(it->second) < TOL ){
      _bndrem += it->second * (2.*Op<T>::zeroone()-1.);
      it = _coefmon.erase( it );
      continue;
    }
    ++it;
  }
  return *this;
}

template <typename T> inline std::ostream&
operator<<
( std::ostream&out, const SCVar<T>&CV )
{
  unsigned IDISP = CV._CM? CV._CM->options.DISPLAY_DIGITS: 5;
  out << std::endl << std::scientific << std::setprecision(IDISP)
      << std::right;

  // Sparse multivariate polynomial
  for( auto it=CV._coefmon.begin(); it!=CV._coefmon.end(); ++it ){
    out << std::right << std::setw(IDISP+7) << it->second << "   ";
    for( auto ie=it->first.second.begin(); ie!=it->first.second.end(); ++ie )
      out << "T" << ie->second << "[" << ie->first << "] ";
    out << std::endl;
  }

  // Remainder term
  out << std::right << "   R     =  " << CV._bndrem
      << std::endl;

  // Range bounder
  out << std::right << "   B     =  " << CV.B()
      << std::endl;

  return out;
}

template <typename T> inline SCVar<T>
operator+
( const SCVar<T>&CV )
{
  return CV;
}

template <typename T> template <typename U> inline SCVar<T>&
SCVar<T>::operator+=
( const SCVar<U>&CV )
{
  if( CV._CM && !_CM ){
    SCVar<T> CV2( *this );
    *this = CV;
    return *this += CV2;
  }

  if( _CM && CV._CM && _CM != CV._CM )
    throw typename SCModel<T>::Exceptions( SCModel<T>::Exceptions::SCMODEL );

  for( auto it=CV._coefmon.begin(); it!=CV._coefmon.end(); ++it ){
    // No warm-start for insert unfortunately...
    auto pt = _coefmon.insert( *it );
    if( !pt.second ) pt.first->second += it->second;
  }
  _bndrem += CV._bndrem;
  _unset_bndpol();
  if( _bndT && CV._bndT ) *_bndT += *CV._bndT;
  else if( _bndT && !CV._CM ) *_bndT += CV.B();
  else _unset_bndT();

  return *this;
}

template <typename T, typename U> inline SCVar<T>
operator+
( const SCVar<T>&CV1, const SCVar<U>&CV2 )
{
  SCVar<T> CV3( CV1 );
  CV3 += CV2;
  return CV3;
}

template <typename T> inline SCVar<T>&
SCVar<T>::operator +=
( const double c )
{
  if( isequal( c, 0. ) ) return *this;
  if( _coefmon.empty() || _coefmon.begin()->first.first ) 
    _coefmon.insert( std::make_pair( std::make_pair( 0, std::map<unsigned,unsigned>() ), c ) );
  else
    _coefmon.begin()->second += c;
  if( _bndpol ) *_bndpol += c;
  if( _bndT )   *_bndT += c;
  return *this;
}

template <typename T> inline SCVar<T>
operator+
( const SCVar<T>&CV1, const double c )
{
  SCVar<T> CV3( CV1 );
  CV3 += c;
  return CV3;
}

template <typename T> inline SCVar<T>
operator+
( const double c, const SCVar<T>&CV2 )
{
  SCVar<T> CV3( CV2 );
  CV3 += c;
  return CV3;
}

template <typename T> template <typename U> inline SCVar<T>&
SCVar<T>::operator+=
( const U&I )
{
  _bndrem += I;
  _center();
  if( _bndT ) *_bndT += I;
  return *this;
}

template <typename T> inline SCVar<T>
operator+
( const SCVar<T>&CV1, const T&I )
{
  SCVar<T> CV3( CV1 );
  CV3 += I;
  return CV3;
}

template <typename T> inline SCVar<T>
operator+
( const T&I, const SCVar<T>&CV2 )
{
  SCVar<T> CV3( CV2 );
  CV3 += I;
  return CV3;
}

template <typename T> inline SCVar<T>
operator-
( const SCVar<T>&CV )
{
  SCVar<T> CV2;
  CV2.set( CV._CM );
  for( auto it=CV._coefmon.begin(); it!=CV._coefmon.end(); ++it )
    CV2._coefmon.insert( std::make_pair( it->first, -it->second ) );
  CV2._bndrem = - CV._bndrem;
  if( CV._bndpol ) CV2._set_bndpol( - *CV._bndpol );
  if( CV._bndT )   CV2._set_bndT( - *CV._bndT );
  return CV2;
}

template <typename T> template <typename U> inline SCVar<T>&
SCVar<T>::operator-=
( const SCVar<U>&CV )
{
  if( CV._CM && !_CM ){
    SCVar<T> CV2( *this );
    *this = -CV;
    return *this += CV2;
  }

  if( _CM && CV._CM && _CM != CV._CM )
    throw typename SCModel<T>::Exceptions( SCModel<T>::Exceptions::SCMODEL );

  for( auto it=CV._coefmon.begin(); it!=CV._coefmon.end(); ++it ){
    // No warm-start for insert unfortunately...
    auto pt = _coefmon.insert( std::make_pair( it->first, -it->second ) );
    if( !pt.second ) pt.first->second -= it->second;
  }
  _bndrem -= CV._bndrem;
  _unset_bndpol();
  if( _bndT && CV._bndT ) *_bndT -= *CV._bndT;
  else if( _bndT && !CV._CM ) *_bndT -= CV.B();
  else _unset_bndT();

  return *this;
}

template <typename T, typename U> inline SCVar<T>
operator-
( const SCVar<T>&CV1, const SCVar<U>&CV2 )
{
  SCVar<T> CV3( CV1 );
  CV3 -= CV2;
  return CV3;
}

template <typename T> inline SCVar<T>&
SCVar<T>::operator-=
( const double c )
{
  if( isequal( c, 0. ) ) return *this;
  if( _coefmon.empty() || _coefmon.begin()->first.first ) 
    _coefmon.insert( std::make_pair( std::make_pair( 0, std::map<unsigned,unsigned>() ), -c ) );
  else
    _coefmon.begin()->second -= c;
  if( _bndpol ) *_bndpol -= c;
  if( _bndT )   *_bndT -= c;
  return *this;
}

template <typename T> inline SCVar<T>
operator-
( const SCVar<T>&CV1, const double c )
{
  SCVar<T> CV3( CV1 );
  CV3 -= c;
  return CV3;
}

template <typename T> inline SCVar<T>
operator-
( const double c, const SCVar<T>&CV2 )
{
  SCVar<T> CV3( -CV2 );
  CV3 += c;
  return CV3;
}

template <typename T> template <typename U> inline SCVar<T>&
SCVar<T>::operator-=
( const U&I )
{
  *_bndrem -= I;
  _center();
  if( _bndT ) *_bndT -= I;
  return *this;
}

template <typename T> inline SCVar<T>
operator-
( const SCVar<T>&CV1, const T&I )
{
  SCVar<T> CV3( CV1 );
  CV3 -= I;
  return CV3;
}

template <typename T> inline SCVar<T>
operator-
( const T&I, const SCVar<T>&CV2 )
{
  SCVar<T> CV3( -CV2 );
  CV3 += I;
  return CV3;
}

template <typename T> inline SCVar<T>&
SCVar<T>::operator*=
( const SCVar<T>&CV )
{
   SCVar<T> CV2( *this );
   *this = CV * CV2;
   return *this;
}

template <typename T> inline SCVar<T>
operator*
( const SCVar<T>&CV1, const SCVar<T>&CV2 )
{
  if( !CV2._CM ) return CV1 * CV2.B();
  if( !CV1._CM ) return CV2 * CV1.B();
  //if( &CV1 == &CV2 ) return sqr(CV1);
  if( CV1._CM != CV2._CM )
    throw typename SCModel<T>::Exceptions( SCModel<T>::Exceptions::SCMODEL );

  // Product of polynomial parts
  SCVar<T> CV3;
  CV3.set( CV1._CM );
  double coefrem( 0. );
  CV1._sprod( CV1, CV2, CV3._coefmon, coefrem );
  CV3._bndrem = coefrem * (Op<T>::zeroone()*2.-1.);

  // Remainder propagation
  T P1 = CV1._polybound();
  T P2 = CV2._polybound();
  T R1 = CV3._bndrem + CV1.B() * CV2._bndrem + P2 * CV1._bndrem;
  T R2 = CV3._bndrem + P1 * CV2._bndrem + CV2.B() * CV1._bndrem;
  if( !Op<T>::inter( CV3._bndrem, R1, R2) )
    CV3._bndrem = ( Op<T>::diam(R1) < Op<T>::diam(R2)? R1: R2 );

  // Bounding
  CV3._unset_bndpol();
  if( CV3._CM->options.MIXED_IA ) CV3._set_bndT( CV1.B() * CV2.B() );
  else CV3._unset_bndT();
  
  // std::cout << CV3;
  return CV3;
}

template <typename T> inline SCVar<T>
sqr
( const SCVar<T>&CV )
{
  if( !CV._CM ) return Op<T>::sqr( CV.B() );

  // Squaring of polynomial part
  SCVar<T> CV2;
  CV2.set( CV._CM );
  double coefrem( 0. );
  CV._ssqr( CV, CV2._coefmon, coefrem );
  CV2._bndrem = coefrem * (Op<T>::zeroone()*2.-1.);

  // Remainder propagation
  T PB = CV._polybound();
  CV2._bndrem += ( PB + CV.B() ) * CV._bndrem;

  // Bounding
  CV2._unset_bndpol();
  if( CV2._CM->options.MIXED_IA ) CV2._set_bndT( Op<T>::sqr(CV.B()) );
  else CV2._unset_bndT();

  return CV2;
}

template <typename T> inline SCVar<T>&
SCVar<T>::operator*=
( const double c )
{
  if( isequal( c, 0. ) ){ *this = 0.; return *this; }
  if( isequal( c, 1. ) ) return *this;
  for( auto it=_coefmon.begin(); it!=_coefmon.end(); ++it )
    it->second *= c;
  _bndrem *= c;
  if( _bndpol ) *_bndpol *= c;
  if( _bndT ) *_bndT *= c;
  return *this;
}

template <typename T> inline SCVar<T>
operator*
( const SCVar<T>&CV1, const double c )
{
  SCVar<T> CV3( CV1 );
  CV3 *= c;
  return CV3;
}

template <typename T> inline SCVar<T>
operator*
( const double c, const SCVar<T>&CV2 )
{
  SCVar<T> CV3( CV2 );
  CV3 *= c;
  return CV3;
}

template <typename T> inline SCVar<T>&
SCVar<T>::operator*=
( const T&I )
{
  const double Imid = Op<T>::mid(I);
  T Icur = bound();
  for( auto it=_coefmon.begin(); it!=_coefmon.end(); ++it )
    it->second *= Imid;
  _bndrem *= Imid;
  _bndrem += ( I - Imid ) * Icur;
  _unset_bndpol();
  if( _bndT ) *_bndT *= I;
  return *this;
}

template <typename T> inline SCVar<T>
operator*
( const SCVar<T>&CV1, const T&I )
{
  SCVar<T> CV3( CV1 );
  CV3 *= I;
  return CV3;
}

template <typename T> inline SCVar<T>
operator*
( const T&I, const SCVar<T>&CV2 )
{
  SCVar<T> CV3( CV2 );
  CV3 *= I;
  return CV3;
}

template <typename T> inline SCVar<T>&
SCVar<T>::operator /=
( const SCVar<T>&CV )
{
   *this *= inv(CV);
   return *this;
}

template <typename T> inline SCVar<T>
operator /
( const SCVar<T>&CV1, const SCVar<T>&CV2 )
{
  return CV1 * inv(CV2);
}

template <typename T> inline SCVar<T>&
SCVar<T>::operator /=
( const double c )
{
  if( isequal( c, 0. ) )
    throw typename SCModel<T>::Exceptions( SCModel<T>::Exceptions::DIV );
  if( isequal( c, 1. ) ) return *this;
  *this *= (1./c);
  return *this;
}

template <typename T> inline SCVar<T>
operator /
( const SCVar<T>&CV, const double c )
{
  if ( isequal( c, 0. ))
    throw typename SCModel<T>::Exceptions( SCModel<T>::Exceptions::DIV );
  if( isequal( c, 1. ) ) return CV;
  return CV * (1./c);
}

template <typename T> inline SCVar<T>
operator /
( const double c, const SCVar<T>&CV )
{
  if( isequal( c, 0. ) ) return 0.;
  if( isequal( c, 1. ) ) return inv(CV);
  return inv(CV) * c;
}

template <typename T> inline SCVar<T>
inv
( const SCVar<T>&CV )
{
  if( !CV._CM )
    return SCVar<T>( Op<T>::inv( CV.B() ) );
  if ( Op<T>::l(CV.B()) <= 0. && Op<T>::u(CV.B()) >= 0. )
    throw typename SCModel<T>::Exceptions( SCModel<T>::Exceptions::INV );

  SCVar<T> CVI( CV._CM ), CV2( CV._CM );
  double* coefmon = CV._coefinterp();
  CV._interpolation( coefmon, mc::inv );

  double m(Op<T>::mid(CV.B())), r(Op<T>::u(CV.B())-m), rem;
#ifndef MC__SCVAR_FORCE_REM_DERIV
  double ub(0), lb(0);
  for (unsigned i(0); i<=CV.maxord(); i++) {
    ub += coefmon[i];
    lb += std::pow(-1.,i)*coefmon[i];
  }
  rem = std::max(std::fabs(mc::inv(r+m)-ub), std::fabs(mc::inv(m-r)-lb));
#else
  rem = 4.*std::pow(r,double(CV.maxord()+2));
#endif

  CVI = CV._rescale(r,m);
  CV2 = CVI._composition( coefmon );
  CV2 += (2.*Op<T>::zeroone()-1.)*rem;

  if( CV._CM->options.MIXED_IA ) CV2._set_bndT( Op<T>::inv( CV.B() ) );
  return CV2;
}

template <typename T> inline SCVar<T>
sqrt
( const SCVar<T>&CV )
{
  if( !CV._CM )
    return SCVar<T>( Op<T>::sqrt( CV.B() ) );
  if ( Op<T>::l(CV.B()) < 0. )
    throw typename SCModel<T>::Exceptions( SCModel<T>::Exceptions::SQRT );

  SCVar<T> CVI( CV._CM ), CV2( CV._CM );
  double* coefmon = CV._coefinterp();
  CV._interpolation( coefmon, std::sqrt );

  double b(Op<T>::mid(CV.B())), a(Op<T>::u(CV.B())-b), rem, ub(0), lb(0);
  for (unsigned i(0); i<=CV.maxord(); i++) {
    ub += coefmon[i];
    lb += i%2? -coefmon[i]: coefmon[i];
  }
  rem = std::max(std::fabs(std::sqrt(a+b)-ub), std::fabs(std::sqrt(b-a)-lb));

  CVI = CV._rescale(a,b);
  CV2 = CVI._composition( coefmon );
  CV2 += (2.*Op<T>::zeroone()-1.)*rem;

  if( CV._CM->options.MIXED_IA ) CV2._set_bndT( Op<T>::sqrt( CV.B() ) );
  return CV2;
}

template <typename T> inline SCVar<T>
exp
( const SCVar<T>&CV )
{ 
  if( !CV._CM )
    return SCVar<T>( Op<T>::exp( CV.B() ) );

  SCVar<T> CVI( CV._CM ), CV2( CV._CM );
  double* coefmon = CV._coefinterp();
  CV._interpolation( coefmon, std::exp );

  double m(Op<T>::mid(CV.B())), r(Op<T>::u(CV.B())-m), rem;
#ifndef MC__SCVAR_FORCE_REM_DERIV
  double ub(0), lb(0);
  for (unsigned i(0); i<=CV.maxord(); i++) {
    ub += coefmon[i];
    lb += std::pow(-1.,i)*coefmon[i];
  }
  rem = std::max(std::fabs(std::exp(r+m)-ub), std::fabs(std::exp(m-r)-lb));
#else
  double fact(1);
  for (unsigned i(1); i<=CV.maxord()+1; i++) fact *= double(i);
  double M = Op<T>::abs(Op<T>::exp(CV.B()));
  rem = 2.*M*std::pow(r/2.,double(CV.maxord()+1))/fact;
#endif

  CVI = CV._rescale(r,m);
  CV2 = CVI._composition( coefmon );
  CV2 += (2.*Op<T>::zeroone()-1.)*rem;

  if( CV._CM->options.MIXED_IA ) CV2._set_bndT( Op<T>::exp( CV.B() ) );
  return CV2;
}

template <typename T> inline SCVar<T>
log
( const SCVar<T>&CV )
{
  if( !CV._CM )
    return SCVar<T>( Op<T>::log( CV.B() ) );
  if ( Op<T>::l(CV.B()) <= 0. )
    throw typename SCModel<T>::Exceptions( SCModel<T>::Exceptions::LOG );

  SCVar<T> CVI( CV._CM ), CV2( CV._CM );
  double* coefmon = CV._coefinterp();
  CV._interpolation( coefmon, std::log );

  double b(Op<T>::mid(CV.B())), a(Op<T>::u(CV.B())-b), rem, ub(0), lb(0);
  for (unsigned i(0); i<=CV.maxord(); i++) {
    ub += coefmon[i];
    lb += std::pow(-1.,i)*coefmon[i];
  }
  rem = std::max(std::fabs(std::log(a+b)-ub), std::fabs(std::log(b-a)-lb));

  CVI = CV._rescale(a,b);
  CV2 = CVI._composition( coefmon );
  CV2 += (2.*Op<T>::zeroone()-1.)*rem;

  if( CV._CM->options.MIXED_IA ) CV2._set_bndT( Op<T>::log( CV.B() ) );
  return CV2;
}

template <typename T> inline SCVar<T>
xlog
( const SCVar<T>&CV )
{
  return CV * log( CV );
}

template <typename T> inline SCVar<T>
pow
( const SCVar<T>&CV, const int n )
{
  if( !CV._CM )
    return SCVar<T>( Op<T>::pow( CV.B(), n ) );

  if( n < 0 ) return pow( inv( CV ), -n );
  SCVar<T> CV2( CV._CM->_intpow( CV, n ) );
  if( CV._CM->options.MIXED_IA ) CV2._set_bndT( Op<T>::pow( CV.B(), n ) );
  return CV2;
}

template <typename T> inline SCVar<T>
pow
( const SCVar<T> &CV, const double a )
{
  return exp( a * log( CV ) );
}

template <typename T> inline SCVar<T>
pow
( const SCVar<T> &CV1, const SCVar<T> &CV2 )
{
  return exp( CV2 * log( CV1 ) );
}

template <typename T> inline SCVar<T>
pow
( const double a, const SCVar<T> &CV )
{
  return exp( CV * std::log( a ) );
}

template <typename T> inline SCVar<T>
monomial
(const unsigned n, const SCVar<T>*CV, const int*k)
{
  if( n == 0 ){
    return 1.;
  }
  if( n == 1 ){
    return pow( CV[0], k[0] );
  }
  return pow( CV[0], k[0] ) * monomial( n-1, CV+1, k+1 );
}

template <typename T> inline SCVar<T>
cheb
( const SCVar<T> &CV, const unsigned n )
{
  switch( n ){
    case 0:  return 1.;
    case 1:  return CV;
    default: break;
  }
  SCVar<T> CV2( 2.*(CV*cheb(CV,n-1))-cheb(CV,n-2) );
  if( CV._CM->options.MIXED_IA ) CV2._set_bndT( Op<T>::cheb( CV.B(), n ) );
  return CV2;
}

template <typename T> inline SCVar<T>
cos
( const SCVar<T> &CV )
{
  if( !CV._CM )
    return SCVar<T>( Op<T>::cos( CV.B() ) );

  SCVar<T> CVI( CV._CM ), CV2( CV._CM );
  double* coefmon = CV._coefinterp();
  CV._interpolation( coefmon, std::cos );

  double m(Op<T>::mid(CV.B())), r(Op<T>::u(CV.B())-m), rem, fact(1);
  for (unsigned i(1); i<=CV.maxord()+1; i++) fact *= double(i);
  double M = CV.maxord()%2? Op<T>::abs(Op<T>::cos(CV.B())):
                          Op<T>::abs(Op<T>::sin(CV.B()));
  rem = 2.*M*std::pow(r/2.,double(CV.maxord()+1))/fact;

  CVI = CV._rescale(r,m);
  CV2 = CVI._composition( coefmon );
  CV2 += (2.*Op<T>::zeroone()-1.)*rem;

  if( CV._CM->options.MIXED_IA ) CV2._set_bndT( Op<T>::cos( CV.B() ) );
  return CV2;
}

template <typename T> inline SCVar<T>
sin
( const SCVar<T> &CV )
{
  return cos( CV - PI/2. );
}

template <typename T> inline SCVar<T>
acos
( const SCVar<T> &CV )
{
  if( !CV._CM )
    return SCVar<T>( Op<T>::acos( CV.B() ) );
  if ( Op<T>::l(CV.B()) < -1. && Op<T>::u(CV.B()) > 1. )
    throw typename SCModel<T>::Exceptions( SCModel<T>::Exceptions::ACOS );

  SCVar<T> CVI( CV._CM ), CV2( CV._CM );
  // INCORRECT AS IMPLEMENTED -- NEEDS FIXING
  throw typename SCModel<T>::Exceptions( SCModel<T>::Exceptions::UNDEF );

  double coefmon[CV.maxord()+1];
  double b(Op<T>::mid(CV.B())), a(Op<T>::u(CV.B())-b), rem, ub(-4.*a/PI);
  coefmon[0] = 0.5*PI*a+b;
  coefmon[1] = ub;
  for (unsigned i(3); i<=CV.maxord(); i+=2) {
    coefmon[i-1] = 0.;
    coefmon[i] = coefmon[1]/std::pow(double(i),2.);
    ub += coefmon[i];
  }
  if (CV.maxord()%2==0) coefmon[CV.maxord()] = 0.;
  rem = a*PI/6. + ub;

  CVI = CV._rescale(a,b);
  CV2 = CVI._composition( coefmon );
  CV2 += (2.*Op<T>::zeroone()-1.)*rem;

  if( CV._CM->options.MIXED_IA ) CV2._set_bndT( Op<T>::acos(CV.B()) );
  return CV2;
}

template <typename T> inline SCVar<T>
asin
( const SCVar<T> &CV )
{
  return PI/2. - acos( CV );
}

template <typename T> inline SCVar<T>
tan
( const SCVar<T> &CV )
{
  return sin( CV ) / cos( CV );
}

template <typename T> inline SCVar<T>
atan
( const SCVar<T> &CV )
{
  return asin( CV / sqrt( sqr( CV ) + 1. ) );
}

template <typename T> inline SCVar<T>
fabs
( const SCVar<T> &CV )
{
  if( !CV._CM )
    return SCVar<T>( Op<T>::fabs( CV.B() ) );
  if ( Op<T>::l(CV.B()) >= 0. )
    return CV;
  if ( Op<T>::u(CV.B()) <= 0. )
    return -CV;

#ifdef MC__SCVAR_FABS_SQRT
  SCVar<T> CV2( sqrt( sqr(CV) ) );
  if( CV._CM->options.MIXED_IA ) CV2._set_bndT( Op<T>::fabs( CV.B() ) );
  return CV2;

#else
  if( CV.maxord() < 2 ){
    SCVar<T> CV2( sqrt( sqr(CV) ) );
    if( CV._CM->options.MIXED_IA ) CV2._set_bndT( Op<T>::fabs( CV.B() ) );
    return CV2;
  }

  SCVar<T> CVI( CV._CM ), CV2( CV._CM );
  double* coefmon = CV._coefinterp();
  CV._interpolation( coefmon, std::fabs );

  double m(Op<T>::mid(CV.B())), r(Op<T>::u(CV.B())-m), rem, fact(1);
  for (unsigned i(1); i<=CV.maxord()+1; i++) fact *= double(i);
  rem = 2./mc::PI*r/double(CV.maxord()-1);
  //rem = 4.*std::pow(a/2.,double(CV.maxord()+1))/fact;

  CVI = CV._rescale(r,m);
  CV2 = CVI._composition( coefmon );
  CV2 += (2.*Op<T>::zeroone()-1.)*rem;

  if( CV._CM->options.MIXED_IA ) CV2._set_bndT( Op<T>::fabs( CV.B() ) );
  return CV2;
#endif
}

template <typename T> inline SCVar<T>
hull
( const SCVar<T>&CV1, const SCVar<T>&CV2 )
{
  // Neither operands associated to SCModel -- Make intersection in T type     
  if( !CV1._CM && !CV2._CM ){
    T R1 = CV1.B();
    T R2 = CV2.B();
    return Op<T>::hull(R1, R2);
  }

  // First operand not associated to SCModel
  else if( !CV1._CM )
    return hull( CV2, CV1 );

  // Second operand not associated to SCModel
  else if( !CV2._CM ){
    SCVar<T> CVR = CV1.P();
    return CVR + Op<T>::hull( CV1.R(), CV2._coefmon[0]+CV2._bndrem-CVR.B() );
  }

  // SCModel for first and second operands are inconsistent
  else if( CV1._CM != CV2._CM )
    throw typename SCModel<T>::Exceptions( SCModel<T>::Exceptions::SCMODEL );

  // Perform union
  SCVar<T> CV1C( CV1 ), CV2C( CV2 );
  const double eta = CV1._CM->options.REF_POLY;
  T R1C = CV1C.C().R(), R2C = CV2C.C().R(); 
  CV1C.set(T(0.));
  CV2C.set(T(0.));
  T BCVD = (CV1C-CV2C).B();
  return (1.-eta)*CV1C + eta*CV2C + Op<T>::hull( R1C+eta*BCVD, R2C+(eta-1.)*BCVD );
}

template <typename T> inline bool
inter
( SCVar<T>&CVR, const SCVar<T>&CV1, const SCVar<T>&CV2 )
{
  // Neither operands associated to SCModel -- Make intersection in T type     
  if( !CV1._CM && !CV2._CM ){
    T R1 = CV1.B();
    T R2 = CV2.B();
    T RR( 0. );
    bool flag = Op<T>::inter(RR, R1, R2);
    CVR = RR;
    return flag;
  }

  // First operand not associated to SCModel
  else if( !CV1._CM )
    return inter( CVR, CV2, CV1 );

  // Second operand not associated to SCModel
  else if( !CV2._CM ){
    // First intersect in T arithmetic
    T B2 = CV2.B(), BR;
    if( CV1._CM->options.MIXED_IA && !Op<T>::inter( BR, CV1.B(), B2 ) )
      return false;

    // Perform intersection in PM arithmetic
    T R1 = CV1.R();
    CVR = CV1.P();
    if( !Op<T>::inter(CVR._bndrem, R1, B2-CVR.B()) )
      return false;
    CVR._center();

    if( CVR._CM->options.MIXED_IA ) CVR._set_bndT( BR );
    else CVR._unset_bndT();
    return true;
  }

  // SCModel for first and second operands are inconsistent
  else if( CV1._CM != CV2._CM )
    throw typename SCModel<T>::Exceptions( SCModel<T>::Exceptions::SCMODEL );

  // First intersect in T arithmetic
  T BR;
  if( CV1._CM->options.MIXED_IA && !Op<T>::inter( BR, CV1.B(), CV2.B() ) )
    return false;

  // Perform intersection in PM arithmetic
  SCVar<T> CV1C( CV1 ), CV2C( CV2 );
  const double eta = CV1._CM->options.REF_POLY;
  T R1C = CV1C.C().R(), R2C = CV2C.C().R(); 
  CV1C.set(T(0.));
  CV2C.set(T(0.));
  CVR = (1.-eta)*CV1C + eta*CV2C;
  CV1C -= CV2C;
  T BCVD = CV1C.B();
  if( !Op<T>::inter( CVR._bndrem, R1C+eta*BCVD, R2C+(eta-1.)*BCVD ) )
    return false;
  CVR._center();

  if( CVR._CM->options.MIXED_IA ) CVR._set_bndT( BR );
  else CVR._unset_bndT();
  return true;
}

} // namespace mc

#include "mcop.hpp"

namespace mc
{

//! @brief C++ structure for specialization of the mc::Op templated structure for use of mc::SCVar in DAG evaluation and as template parameter in other MC++ types
template< typename T > struct Op< mc::SCVar<T> >
{
  typedef mc::SCVar<T> CV;
  static CV point( const double c ) { return CV(c); }
  static CV zeroone() { return CV( mc::Op<T>::zeroone() ); }
  static void I(CV& x, const CV&y) { x = y; }
  static double l(const CV& x) { return mc::Op<T>::l(x.B()); }
  static double u(const CV& x) { return mc::Op<T>::u(x.B()); }
  static double abs (const CV& x) { return mc::Op<T>::abs(x.B());  }
  static double mid (const CV& x) { return mc::Op<T>::mid(x.B());  }
  static double diam(const CV& x) { return mc::Op<T>::diam(x.B()); }
  static CV inv (const CV& x) { return mc::inv(x);  }
  static CV sqr (const CV& x) { return mc::sqr(x);  }
  static CV sqrt(const CV& x) { return mc::sqrt(x); }
  static CV log (const CV& x) { return mc::log(x);  }
  static CV xlog(const CV& x) { return x*mc::log(x); }
  static CV fabs(const CV& x) { return mc::fabs(x); }
  static CV exp (const CV& x) { return mc::exp(x);  }
  static CV sin (const CV& x) { return mc::sin(x);  }
  static CV cos (const CV& x) { return mc::cos(x);  }
  static CV tan (const CV& x) { return mc::tan(x);  }
  static CV asin(const CV& x) { return mc::asin(x); }
  static CV acos(const CV& x) { return mc::acos(x); }
  static CV atan(const CV& x) { return mc::atan(x); }
  static CV erf (const CV& x) { throw typename mc::SCModel<T>::Exceptions( SCModel<T>::Exceptions::UNDEF ); }
  static CV erfc(const CV& x) { throw typename mc::SCModel<T>::Exceptions( SCModel<T>::Exceptions::UNDEF ); }
  static CV fstep(const CV& x) { return CV( mc::Op<T>::fstep(x.B()) ); }
  static CV bstep(const CV& x) { return CV( mc::Op<T>::bstep(x.B()) ); }
  static CV hull(const CV& x, const CV& y) { return mc::hull(x,y); }
  static CV min (const CV& x, const CV& y) { return mc::Op<T>::min(x.B(),y.B());  }
  static CV max (const CV& x, const CV& y) { return mc::Op<T>::max(x.B(),y.B());  }
  static CV arh (const CV& x, const double k) { return mc::exp(-k/x); }
  static CV cheb(const CV& x, const unsigned n) { return mc::cheb(x,n); }
  template <typename X, typename Y> static CV pow(const X& x, const Y& y) { return mc::pow(x,y); }
  static CV monomial (const unsigned n, const T* x, const int* k) { return mc::monomial(n,x,k); }
  static bool inter(CV& xIy, const CV& x, const CV& y) { return mc::inter(xIy,x,y); }
  static bool eq(const CV& x, const CV& y) { return mc::Op<T>::eq(x.B(),y.B()); }
  static bool ne(const CV& x, const CV& y) { return mc::Op<T>::ne(x.B(),y.B()); }
  static bool lt(const CV& x, const CV& y) { return mc::Op<T>::lt(x.B(),y.B()); }
  static bool le(const CV& x, const CV& y) { return mc::Op<T>::le(x.B(),y.B()); }
  static bool gt(const CV& x, const CV& y) { return mc::Op<T>::gt(x.B(),y.B()); }
  static bool ge(const CV& x, const CV& y) { return mc::Op<T>::ge(x.B(),y.B()); }
};

} // namespace mc

#endif

