// Copyright (C) 2009-2016 Benoit Chachuat, Imperial College London.
// All Rights Reserved.
// This code is published under the Eclipse Public License.

/*!
\page page_CHEBYSHEV Chebyshev Model Arithmetic for Factorable Functions
\author Jai Rajyaguru, Mario E. Villanueva, Beno&icirc;t Chachuat

A \f$q\f$th-order Chebyshev model of a Lipschitz-continuous function \f$f:\mathbb{R}^n\to\mathbb{R}\f$ on the domain \f$D\f$, consists of a \f$q^{\rm th}\f$-order multivariate polynomial \f$\mathcal P\f$ in Chebyshev basis , plus a remainder term \f$\mathcal R\f$, so that
\f{align*}
  f({x}) \in \mathcal P({x}) \oplus \mathcal R, \quad \forall {x}\in D.
\f}
The polynomial part \f$\mathcal P\f$ is propagated symbolically and accounts for functional dependencies. The remainder term \f$\mathcal R\f$, on the other hand, is traditionally computed using interval analysis [Brisebarre & Joldes, 2010]; see figure below. More generally, convex/concave bounds or an ellipsoidal enclosure can be computed for the remainder term of vector-valued functions too. In particular, it can be established that the remainder term has convergence order (no less than) \f$q+1\f$ with respect to the diameter of the domain set \f$D\f$ under mild conditions [Bompadre <I>et al.</I>, 2012].

<CENTER><TABLE BORDER=0>
<TR>
<TD>\image html Chebyshev_model.png</TD>
</TR>
</TABLE></CENTER>

The classes mc::CModel and mc::CVar provide an implementation of Chebyshev model arithmetic. We note that mc::CModel / mc::CVar is <b>not a verified implementation</b> in the sense that rounding errors are not accounted for in propagating the coefficients in the multivariate polynomial part, which are treated as floating-point numbers.

The implementation of mc::CModel and mc::CVar relies on the operator/function overloading mechanism of C++. This makes the computation of Chebyshev models both simple and intuitive, similar to computing function values in real arithmetics or function bounds in interval arithmetic (see \ref page_INTERVAL). Moreover, mc::CVar can be used as the template parameter of other available types in MC++; for instance, mc::CVar can be used in order to propagate the underlying interval bounds in mc::McCormick. Likewise, mc::CVar can be used as the template parameter of the types fadbad::F, fadbad::B and fadbad::T of <A href="http://www.fadbad.com/fadbad.html">FADBAD++</A> for computing Chebyshev models of either the partial derivatives or the Chebyshev coefficients of a factorable function (see \ref sec_CHEBYSHEV_fadbad).

mc::CModel and mc::CVar themselves are templated in the type used to propagate bounds on the remainder term. By default, mc::CModel and mc::CVar can be used with the non-verified interval type mc::Interval of MC++. For reliability, however, it is strongly recommended to use verified interval arithmetic such as <A href="http://www.ti3.tu-harburg.de/Software/PROFILEnglisch.html">PROFIL</A> (header file <tt>mcprofil.hpp</tt>) or <A href="http://www.math.uni-wuppertal.de/~xsc/software/filib.html">FILIB++</A> (header file <tt>mcfilib.hpp</tt>). As already noted, convex/concave bounds on the remainder term can also be propagated by using the type mc::McCormick of MC++, thereby enabling McCormick-Chebyshev models.

As well as propagating Chebyshev models for factorable functions, mc::CModel and mc::CVar provide support for computing bounds on the Chebyshev model range (multivariate polynomial part). We note that computing exact bounds for multivariate polynomials is a hard problem in general. Instead, a number of computationally tractable, yet typically conservative, bounding approaches are implemented in mc::CModel and mc::CVar, which include:
- Bounding every monomial term independently and adding these bounds;
- Bounding the first- and diagonal second-order terms exactly and adding bounds for the second-order off-diagonal and higher-order terms computed independently [Lin & Stadtherr, 2007];
- Bounding the terms up to order 2 based on an eigenvalue decomposition of the corresponding Hessian matrix and adding bounds for the higher-order terms computed independently;
- Expressing the multivariate polynomial in Bernstein basis, thereby providing bounds as the minimum/maximum among all Bernstein coefficients [Lin & Rokne, 1995; 1996].
.

Examples of Chebyshev models (blue lines) constructed with mc::CModel and mc::CVar are shown on the figure below for the factorable function \f$f(x)=x \exp(-x^2)\f$ (red line) for \f$x\in [-0.5,1]\f$. Also shown on these plots are the interval bounds computed from the Chebyshev models.

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
      typedef mc::CModel<I> CM;
      typedef mc::CVar<I> CV;
\endcode

First, the number of independent variables in the factorable function (\f$x\f$ and \f$y\f$ here) as well as the order of the Chebyshev model (4th order here) are specified by defining an mc::CModel object as:

\code
      CM mod( 2, 4 );
\endcode

Next, the variables \f$x\f$ and \f$y\f$ are defined as follows:

\code
      CV X( &mod, 0, I(1.,2.) );
      CV Y( &mod, 1, I(0.,1.) );
\endcode

Essentially, the first line means that <tt>X</tt> is a variable of class mc::CVar, participating in the Chebyshev model <tt>mod</tt>, belonging to the interval \f$[1,2]\f$, and having index 0 (indexing in C/C++ start at 0 by convention!). The same holds for the Chebyshev variable <tt>Y</tt>, participating in the model <tt>mod</tt>, belonging to the interval \f$[0,1]\f$, and having index 1.

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

See the documentations of mc::CModel and mc::CVar for a complete list of member functions. 


\section sec_CHEBYSHEV_fct Which functions are overloaded for Chebyshev model arithmetic?

mc::CVar overloads the usual functions <tt>exp</tt>, <tt>log</tt>, <tt>sqr</tt>, <tt>sqrt</tt>, <tt>pow</tt>, <tt>inv</tt>, <tt>cos</tt>, <tt>sin</tt>, <tt>tan</tt>, <tt>acos</tt>, <tt>asin</tt>, <tt>atan</tt>. Unlike mc::Interval and mc::McCormick, the functions <tt>min</tt>, <tt>max</tt> and <tt>fabs</tt> are not overloaded in mc::CVar as they are nonsmooth. Moreover, mc::CVar defines the following functions:
- <tt>inter(x,y,z)</tt>, computing a Chebyshev model of the intersection \f$x = y\cap z\f$ of two Chebyshev models and returning true/false if the intersection is nonempty/empty. With Chebyshev models \f$\mathcal P_y\oplus\mathcal R_y\f$ and \f$\mathcal P_z\oplus\mathcal R_z\f$, this intersection is computed as follows:
\f{align*}
  \mathcal P_{x} =\ & (1-\eta) \mathcal P_y^{\rm C} + \eta \mathcal P_z^{\rm C}\\
  \mathcal R_{x} =\ & [\mathcal R_y^{\rm C}\oplus\eta\mathcal{B}(\mathcal P_y^{\rm C}-\mathcal P_z^{\rm C})] \cap [\mathcal R_z^{\rm C}\oplus (1-\eta)\mathcal{B}(\mathcal P_z^{\rm C}-\mathcal P_y^{\rm C})]\,.
\f}
with \f$\mathcal{B}(\cdot)\f$ the Chebyshev model range bounder, and \f$\eta\f$ a real scalar in \f$[0,1]\f$. Choosing \f$\eta=1\f$ amounts to setting the polynomial part \f$\mathcal P_{x}\f$ as \f$\mathcal P_y\f$, whereas \f$\eta=0\f$ sets \f$\mathcal P_{x}\f$ as \f$\mathcal P_z\f$. The parameter \f$\eta\f$ can be defined in mc::CModel::Options::REF_POLY.
- <tt>hull(x,y)</tt>, computing a Chebyshev model of the union \f$x = y\cup z\f$ of two Chebyshev models. With Chebyshev models \f$\mathcal P_y\oplus\mathcal R_y\f$ and \f$\mathcal P_z\oplus\mathcal R_z\f$, this union is computed as follows:
\f{align*}
  \mathcal P_{x} =\ & (1-\eta) \mathcal P_y^{\rm C} + \eta \mathcal P_z^{\rm C}\\
  \mathcal R_{x} =\ & {\rm hull}\{\mathcal R_y^{\rm C}\oplus\eta\mathcal{B}(\mathcal P_y^{\rm C}-\mathcal P_z^{\rm C}), \mathcal R_z^{\rm C}\oplus (1-\eta)\mathcal{B}(\mathcal P_z^{\rm C}-\mathcal P_y^{\rm C})\}\,.
\f}
with \f$\mathcal{B}(\cdot)\f$ and \f$\eta\f$ as previously.


\section sec_CHEBYSHEV_opt How are the options set for the computation of a Chebyshev model?

The class mc::CModel has a public member called mc::CModel::options that can be used to set/modify the options; e.g.,

\code
      model.options.BOUNDER_TYPE = CM::Options::EIGEN;
      model.options.SCALE_VARIABLES = true;
\endcode

The available options are the following:

<TABLE border="1">
<CAPTION><EM>Options in mc::CModel::Options: name, type and description</EM></CAPTION>
     <TR><TH><b>Name</b>  <TD><b>Type</b><TD><b>Default</b>
         <TD><b>Description</b>
     <TR><TH><tt>BOUNDER_TYPE</tt> <TD><tt>mc::CModel::Options::BOUNDER</tt> <TD>mc::CModel::Options::LSB
         <TD>Chebyshev model range bounder.
     <TR><TH><tt>BOUNDER_ORDER</tt> <TD><tt>unsigned int</tt> <TD>0
         <TD>Order of Bernstein polynomial for Chebyshev model range bounding, when mc::CModel::options::BOUNDER_TYPE = mc::CModel::options::BERNSTEIN is selected. Only values greater than the actual Chebyshev model order are accounted for; see [Lin & Rokne, 1996].
     <TR><TH><tt>REF_POLY</tt> <TD><tt>double</tt> <TD>0.
         <TD>Scalar in \f$[0,1]\f$ related to the choice of the polynomial part in the overloaded functions mc::inter and mc::hull (see \ref sec_CHEBYSHEV_fct). A value of 0. amounts to selecting the polynomial part of the left operand, whereas a value of 1. selects the right operand.
     <TR><TH><tt>DISPLAY_DIGITS</tt> <TD><tt>unsigned int</tt> <TD>5
         <TD>Number of digits in output stream for Chebyshev model coefficients.
</TABLE>


\section sec_CM_err Errors What errors can I encounter during computation of a Chebyshev model?

Errors are managed based on the exception handling mechanism of the C++ language. Each time an error is encountered, a class object of type mc::CModel::Exceptions is thrown, which contains the type of error. It is the user's responsibility to test whether an exception was thrown during the computation of a Chebyshev model, and then make the appropriate changes. Should an exception be thrown and not caught by the calling program, the execution will abort.

Possible errors encountered during the computation of a Chebyshev model are:

<TABLE border="1">
<CAPTION><EM>Errors during the Computation of a Chebyshev Model</EM></CAPTION>
     <TR><TH><b>Number</b> <TD><b>Description</b>
     <TR><TH><tt>1</tt> <TD>Division by zero
     <TR><TH><tt>2</tt> <TD>Failed to compute eigenvalue decomposition in range bounder CModel::Options::EIGEN
     <TR><TH><tt>3</tt> <TD>Failed to compute the maximum gap between a univariate term and its Bernstein model
     <TR><TH><tt>-1</tt> <TD>Number of variable in Chebyshev model must be nonzero
     <TR><TH><tt>-2</tt> <TD>Failed to construct Chebyshev variable
     <TR><TH><tt>-3</tt> <TD>Chebyshev model bound does not intersect with bound in template parameter arithmetic
     <TR><TH><tt>-4</tt> <TD>Operation between Chebyshev variables linked to different Chebyshev models
     <TR><TH><tt>-5</tt> <TD>Maximum size of Chebyshev model reached (monomials indexed as unsigned int)
     <TR><TH><tt>-33</tt> <TD>Feature not yet implemented in mc::CModel
</TABLE>

Moreover, exceptions may be thrown by the template parameter class itself.


\section sec_CM_refs References

- Brisebarre, N., and M. Joldes, <A href="http://hal.archives-ouvertes.fr/docs/00/48/17/37/PDF/RRLIP2010-13.pdf">Chebyshev Interpolation Polynomial-based Tools for Rigorous Computing</A>, <i>Research Report No RR2010-13</i>, Ecole Normale Sup&eaccute;rieure de Lyon, Unit&eaccute; Mixte de Recherche CNRS-INRIA-ENS LYON-UCBL No 5668, 2010
.
*/

#ifndef MC__CMODEL_H
#define MC__CMODEL_H

#include "polymodel.hpp"
#include "mcop.hpp"

#undef  MC__CMODEL_DEBUG
#undef  MC__CMODEL_DEBUG_SCALE
#define MC__CMODEL_CHECK
#undef  MC__CMODEL_CHECK_PMODEL
#undef  MC__CVAR_DEBUG_EXP
#undef  MC__CVAR_DEBUG_BERNSTEIN

//#undef MC__CVAR_FABS_SQRT
//#undef MC__CVAR_FORCE_REM_DERIV
//#undef MC__CVAR_SPARSE_PRODUCT_FULL

namespace mc
{

template <typename T> class CVar;

//! @brief C++ class for the computation of Chebyshev models of factorable function - Chebyshev model environment
////////////////////////////////////////////////////////////////////////
//! mc::CModel is a C++ class for definition of Chebyshev model
//! environment, derived from mc::PolyModel. Propagation of Chebyshev 
//! models for factorable functions is via the C++ class mc::CVar. The
//! template parameter corresponds to the type used to propagate the
//! remainder bound.
////////////////////////////////////////////////////////////////////////
template <typename T>
class CModel
: public PolyModel
////////////////////////////////////////////////////////////////////////
{
  friend class CVar<T>;
  template <typename U> friend class CModel;

  template <typename U> friend CVar<U> pow
    ( const CVar<U>&, const int );

public:

  /** @addtogroup CHEBYSHEV Chebyshev Model Arithmetic for Factorable Functions
   *  @{
   */
  //! @brief Constructor of Chebyshev model environment for <tt>nvar</tt> variables and order <tt>nord</tt>
  CModel
    ( const unsigned nvar, const unsigned nord, const bool sparse=false )
    : PolyModel( nvar, nord, sparse )
    { _size(); }

  //! @brief Destructor of Chebyshev model environment
  ~CModel()
    { _cleanup(); }

  //! @brief Get Chebyshev basis functions in U arithmetic for variable array <a>X</a>
  template <typename U> U** get_basis
    ( const unsigned nord, const U*X, const bool scaled=false ) const;
 
  //! @brief Get Chebyshev monomial bounds in U arithmetic for variable array <a>X</a>
  // template <typename U> std::pair<unsigned,U*> get_bndmon
  //  ( const unsigned nord, const U*X, const bool scaled=false ) const;

  //! @brief Get Chebyshev monomial bounds in U arithmetic for variable array <a>X</a>
  template <typename U> void get_bndmon
    ( const unsigned nord, U*bndmon, const U*X, const bool scaled=false ) const;

  //! @brief Get Chebyshev monomial bounds in U arithmetic for variable array <a>X</a> and monomial indexes in <a>ndxmon</a>
  template <typename U> void get_bndmon
    ( const unsigned nord, U*bndmon, const std::set<unsigned>&ndxmon,
      const U*X, const bool scaled=false ) const;

  //! @brief Polynomial range bounder using specified bounder <a>type</a> with basis functions <a>bndbasis</a> in U arithmetic and monomial coefficients <a>coefmon</a> in C arithmetic
  template <typename C, typename U> U get_bound
    ( const C*coefmon, const U*const*bndbasis, const U*bndrem, const int type,
      const std::set<unsigned>&ndxmon=std::set<unsigned>() )
    { return( bndrem? _polybound( coefmon, bndbasis, type, ndxmon ) + *bndrem:
                      _polybound( coefmon, bndbasis, type, ndxmon ) ); }

  //! @brief Reset the bounds on Chebyshev basis functions
  void reset()
    { _reset(); };  

  //! @brief Lift Chebyshev model by <a>nx</a> extra dimensions with variable bounds <a>X</a>
  CModel<T>* lift
    ( const unsigned nx, const T*X ) const;

  //! @brief Exceptions of mc::CModel
  class Exceptions
  {
  public:
    //! @brief Enumeration type for CModel exception handling
    enum TYPE{
      DIV=1,	//!< Division by zero scalar
      INV,	//!< Inverse operation with zero in range
      LOG,	//!< Log operation with non-positive numbers in range
      SQRT,	//!< Square-root operation with negative numbers in range
      ACOS,	//!< Sine/Cosine inverse operation with range outside [-1,1]
      EIGEN,	//!< Failed to compute eigenvalue decomposition in range bounder CModel::Options::EIGEN
      INIT=-1,	//!< Failed to construct Chebyshev variable
      INCON=-2, //!< Chebyshev model bound does not intersect with bound in template parameter arithmetic
      CMODEL=-3,//!< Operation between Chebyshev variables linked to different Chebyshev models
      INTERNAL = -4,//!< Internal error
      UNDEF=-33 //!< Feature not yet implemented in mc::CModel
    };
    //! @brief Constructor for error <a>ierr</a>
    Exceptions( TYPE ierr ) : _ierr( ierr ){}
    //! @brief Error flag
    int ierr(){ return _ierr; }
    //! @brief Error description
    std::string what(){
      switch( _ierr ){
      case DIV:
        return "mc::CModel\t Division by zero scalar";
      case INV:
        return "mc::CModel\t Inverse operation with zero in range";
      case LOG:
        return "mc::CModel\t Log operation with non-positive numbers in range";
      case SQRT:
        return "mc::CModel\t Square-root operation with negative numbers in range";
      case ACOS:
        return "mc::CModel\t Sine/Cosine inverse operation with range outside [-1,1]";
      case EIGEN:
        return "mc::CModel\t Range bounder with eigenvalue decomposition failed";
      case INIT:
        return "mc::CModel\t Chebyshev variable initialization failed";
      case INCON:
        return "mc::CModel\t Inconsistent bounds with template parameter arithmetic";
      case CMODEL:
        return "mc::CModel\t Operation between Chebyshev variables in different Chebyshev model environment not allowed";
      case UNDEF:
        return "mc::CModel\t Feature not yet implemented in mc::CModel class";
      case INTERNAL:
        return "mc::CModel\t Internal error";
      default:
        return "mc::CModel\t Undocumented error";
      }
    }

  private:
    TYPE _ierr;
  };

  //! @brief Options of mc::CModel
  struct Options
  {
    //! @brief Constructor of mc::CModel::Options
    Options():
      INTERP_EXTRA(0), BOUNDER_TYPE(LSB), BOUNDER_ORDER(0), MIXED_IA(false),
      REF_POLY(0.), DISPLAY_DIGITS(5)
      {}
    //! @brief Copy constructor of mc::CModel::Options
    template <typename U> Options
      ( const U&options )
      : INTERP_EXTRA( options.INTERP_EXTRA ),
        BOUNDER_TYPE( options.BOUNDER_TYPE ),
        BOUNDER_ORDER( options.BOUNDER_ORDER ),
        MIXED_IA( options.MIXED_IA ),
        REF_POLY(options.REF_POLY),
	DISPLAY_DIGITS(options.DISPLAY_DIGITS)
      {}
    //! @brief Assignment of mc::CModel::Options
    template <typename U> Options& operator =
      ( const U&options ){
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
      BERNSTEIN,//!< Bernstein range bounder
      HYBRID	//!< Hybrid LSB + EIGEN range bounder
    };
    //! @brief Extra terms in chebyshev interpolation of univariates: 0-Chebyshev interpolation of order NORD; extra terms allow approximation of Chebyshev truncated series
    unsigned INTERP_EXTRA;
    //! @brief Chebyshev model range bounder - See \ref sec_CHEBYSHEV_opt
    BOUNDER BOUNDER_TYPE;
    //! @brief Order of Bernstein polynomial for Chebyshev model range bounding (no less than Chebyshev model order!). Only if mc::CModel::options::BOUNDER_TYPE is set to mc::CModel::options::BERNSTEIN.
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

  //! @brief Original bounds on variable <tt>ivar</tt>
  const T& bndvar
    ( const unsigned ivar ) const
    { return _bndvar[ivar]; };

  //! @brief Reference point for variable <tt>ivar</tt> in Chebyshev model
  double refvar
    ( const unsigned ivar ) const
    { return _refvar[ivar]; };

  //! @brief Scaling for variable <tt>ivar</tt> in Cheyshev model
  double scalvar
    ( const unsigned ivar ) const
    { return _scalvar[ivar]; };

private:  
  //! @brief Triple array of size <tt>(_nmon+1,<=_nmon,2^_nvar)</tt> with indices of terms from product of two monomial terms <tt>imon=1,...,_nmon</tt> and <tt>jmon=1,...,_nmon</tt> in Chebyshev model
  unsigned ***_prodmon;

  //! @brief Array of size <tt>_nmon</tt> with bounds on Chebyshev monomials <tt>imon=1,...,_nmon</tt>
  T *_bndmon;

  //! @brief Double array of size <tt>(_nvar,_nord+1)</tt> with bounds on Chebyshev basis functions
  T **_bndpow;

  //! @brief Array of size <tt>_nvar</tt> with bounds on original variables <tt>ivar=1,...,_nvar</tt>
  T *_bndvar;

  //! @brief Array of size <tt>_ncoefinterp</tt> with coefficients in Chebyshev interpolant of univariate functions
  double *_coefinterp;

  //! @brief Size of array <tt>_coefinterp</tt>
  unsigned _ncoefinterp;

  //! @brief Internal Chebyshev variable to speed-up computations and reduce dynamic allocation
  CVar<T>* _CV;

  //! @brief Set Chebyshev model order <tt>nord</tt> and number of variables <tt>nvar</tt>
  void _size();
      
  //! @brief Reset the bounds on Chebyshev basis functions
  void _reset();

  //! @brief Clean up the arrays
  void _cleanup();

  //! @brief Populate array <tt>_bndmon</tt>
  void _set_bndmon();

  //! @brief Populate array _prodmon w/ exponents resulting from the product of two monomial terms 1,...,nmon
  void _set_prodmon();

  //! @brief Generate indices of basis functions in product monics recursively
  void _set_prodmon_exp
    ( unsigned int&ivar, const unsigned int*iexp, const unsigned iord,
      unsigned int*iprodexp, unsigned int&nprodmon );

  //! @brief Populate array <tt>_bndpow</tt> for variable <tt>i</tt>
  void _set_bndpow
    ( const unsigned i, const T&X, const double ref, const double scal );

  //! @brief Get Chebyshev basis functions in U arithmetic for variable <a>X</a>
  template <typename U> static U* _get_bndpow
    ( const unsigned nord, const U&X, const double ref, const double scal );

  //! @brief Get Chebyshev basis functions in U arithmetic for [-1,1] scaled variable <a>X</a>
  template <typename U> static U* _get_bndpow
    ( const unsigned nord, const U&X );

  //! @brief Resize array <a>_coefinterp</a> holding coefficients in Chebyshev interpolant of univariate functions
  double* _resize_coefinterp();

  //! @brief Prototype real-valued function for interpolation
  typedef double (puniv)
    ( const double x );

  //! @brief Recursive calculation of nonnegative integer powers
  CVar<T> _intpow
    ( const CVar<T>&CV, const int n ) const;

  //! @brief Construct Chebyshev interpolating polynomial coefficient <a>coefmon</a> for univariate <a>f</a>
  static void _interpolation
    ( double*coefmon, const unsigned nord, const T&X, puniv f );

  //! @brief Apply Chebyshev composition to variable <a>CVI</a> using the coefficients <a>coefmon</a> of the outer function
  template <typename U> static CVar<T> _composition
    ( const U* coefouter, const unsigned nord, const CVar<T>& CVinner );

  //! @brief Polynomial range bounder - Lin & Stadtherr approach
  template <typename C, typename U> U _polybound_LSB
    ( const C*coefmon, const U*bndord, const U*const*bndbasis,
      const std::set<unsigned>&ndxmon=std::set<unsigned>() ) const;

  //! @brief Polynomial range bounder - eigenvalue decomposition approach
  template <typename C, typename U> U _polybound_eigen
    ( const C*coefmon, const U*bndord, const U*const*bndbasis,
      const std::set<unsigned>&ndxmon=std::set<unsigned>() ) const;

  //! @brief Polynomial range bounder - Bernstein approach
  template <typename C, typename U> U _polybound_bernstein
    ( const C*coefmon, const U*bndord, const U*const*bndbasis,
      const std::set<unsigned>&ndxmon=std::set<unsigned>() );

  //! @brief Compute Bernstein coefficient for variable with exponents <tt>jexp</tt>, given coefficients in Chebyshev form <tt>coefmon</tt>, maximum order <tt>maxord</tt> and transformation coefficients <a>trmat</a>
  template <typename C> C _coef_bernstein
    ( const C*coefmon, const unsigned*jexp, const double*trmat,
      const std::set<unsigned>&ndxmon=std::set<unsigned>() ) const;

  //! @brief Compute Bernstein tranformation coefficient into Bernstein basis function <a>j</a> with order <a>n</a> from Chebyshev basis function <a>k</a>
  double _transform_bernstein
    ( const unsigned j, const unsigned k, const unsigned n ) const;

  //! @brief Polynomial range bounder using specified bounder <a>type</a> with basis functions <a>bndbasis</a> in U arithmetic and monomial coefficients <a>coefmon</a> in C arithmetic
  template <typename C, typename U> U _polybound
    ( const C*coefmon, const U*const*bndbasis, const int type,
      const std::set<unsigned>&ndxmon=std::set<unsigned>() );

  //! @brief Scaling of Chebyshev coefficient maps
  void _sscal1D
    ( const std::map<unsigned,double>&CVmap, const double&coefscal,
      std::map<unsigned,double>&CVRmap ) const;

  //! @brief Recursive product of univariate Chebyshev polynomials
  void _sprod1D
    ( const std::vector<std::map<unsigned,double>>&CV1map,
      const std::vector<std::map<unsigned,double>>&CV2map,
      std::map<unsigned,double>&CVRmap, double&coefrem,
      const unsigned ndxmon ) const;

  //! @brief Lifting of Chebyshev coefficient maps
  void _slift1D
    ( const std::map<unsigned,double>&CVmap, const double dscal,
      std::map<unsigned,double>&CVRmap ) const;

  //! @brief Lifting of Chebyshev coefficient maps
  void _slift1D
    ( const std::map<unsigned,double>&CVmap, const double dscal,
      std::map<unsigned,double>&CVRmap, double&coefrem,
      const unsigned ndxvar, const unsigned ndxord, unsigned*iexp ) const;

  //! @brief Lifting of Chebyshev coefficient maps
  void _slift1D
    ( const std::map<unsigned,double>&CVmap, double&coefrem ) const;

  //! @brief Display of recursive univariate Chebyshev polynomials
  void _sdisp1D
    ( const std::vector<std::map<unsigned,double>>&CVmap,
      const unsigned ndxvar, const std::string&CVname="",
      std::ostream&os=std::cout ) const;

  //! @brief Display of recursive univariate Chebyshev polynomials
  void _sdisp1D
    ( const std::map<unsigned,double>&CVmap,
      const unsigned ndxvar, const std::string&CVname="",
      std::ostream&os=std::cout ) const;

  //! @brief Product of multivariate Chebyshev polynomials in sparse format
  void _sprod
    ( const CVar<T>&CV1, const CVar<T>&CV2, double*coefmon,
      std::set<unsigned>&ndxmon, double&coefrem ) const;

  //! @brief Squaring of multivariate Chebyshev polynomials in sparse format
  void _ssqr
    ( const CVar<T>&CV, double*coefmon,
      std::set<unsigned>&ndxmon, double&coefrem ) const;
};

template <typename T> const std::string CModel<T>::Options::BOUNDER_NAME[5]
  = { "NAIVE", "LSB", "EIGEN", "BERNSTEIN", "HYBRID" };

//! @brief C++ class for Chebyshev model computation of factorable function - Chebyshev model propagation
////////////////////////////////////////////////////////////////////////
//! mc::CVar is a C++ class for propagation of Chebyshev models through
//! factorable functions. The template parameter corresponds to the
//! type used in computing the remainder bound.
////////////////////////////////////////////////////////////////////////
template <typename T>
class CVar: public PolyVar<T>
////////////////////////////////////////////////////////////////////////
{
  template <typename U> friend class CVar;
  template <typename U> friend class CModel;

  template <typename U> friend CVar<U> operator-
    ( const CVar<U>& );
  template <typename U> friend CVar<U> operator*
    ( const CVar<U>&, const CVar<U>& );
  template <typename U> friend std::ostream& operator<<
    ( std::ostream&, const CVar<U>& );

  template <typename U> friend U funcptr
    ( const U, const unsigned  );
  template <typename U> friend void interpolation
    ( double*, const CVar<U>&, const unsigned  );
  template <typename U> friend CVar<U> composition
    ( const double*, const CVar<U>& );

  template <typename U> friend CVar<U> inv
    ( const CVar<U>& );
  template <typename U> friend CVar<U> sqr
    ( const CVar<U>& );
  template <typename U> friend CVar<U> sqrt
    ( const CVar<U>& );
  template <typename U> friend CVar<U> exp
    ( const CVar<U>& );
  template <typename U> friend CVar<U> log
    ( const CVar<U>& );
  template <typename U> friend CVar<U> xlog
    ( const CVar<U>& );
  template <typename U> friend CVar<U> pow
    ( const CVar<U>&, const int );
  template <typename U> friend CVar<U> pow
    ( const CVar<U>&, const double );
  template <typename U> friend CVar<U> pow
    ( const double, const CVar<U>& );
  template <typename U> friend CVar<U> pow
    ( const CVar<U>&, const CVar<U>& );
  template <typename U> friend CVar<U> monomial
    ( const unsigned int, const CVar<U>*, const int* );
  template <typename U> friend CVar<U> cheb
    ( const CVar<U>&, const unsigned );
  template <typename U> friend CVar<U> cos
    ( const CVar<U>& );
  template <typename U> friend CVar<U> sin
    ( const CVar<U>& );
  template <typename U> friend CVar<U> tan
    ( const CVar<U>& );
  template <typename U> friend CVar<U> acos
    ( const CVar<U>& );
  template <typename U> friend CVar<U> asin
    ( const CVar<U>& );
  template <typename U> friend CVar<U> atan
    ( const CVar<U>& );
  template <typename U> friend CVar<U> fabs
    ( const CVar<U>& );
  template <typename U> friend CVar<U> hull
    ( const CVar<U>&, const CVar<U>& );
  template <typename U> friend bool inter
    ( CVar<U>&, const CVar<U>&, const CVar<U>& );

public:
  using PolyVar<T>::_coefmon;
  using PolyVar<T>::_bndord;
  using PolyVar<T>::_bndord_uptd;
  using PolyVar<T>::_bndrem;
  using PolyVar<T>::_bndT;
  using PolyVar<T>::_set_bndT;
  using PolyVar<T>::_unset_bndT;
  using PolyVar<T>::_bndpol;
  using PolyVar<T>::_set_bndpol;
  using PolyVar<T>::_unset_bndpol;
  using PolyVar<T>::nvar;
  using PolyVar<T>::nord;
  using PolyVar<T>::nmon;
  using PolyVar<T>::_posord;
  using PolyVar<T>::_expmon;
  using PolyVar<T>::_ndxmon;
  using PolyVar<T>::_loc_expmon;
  using PolyVar<T>::_get_binom;
  using PolyVar<T>::_resize;
  using PolyVar<T>::_set;
  using PolyVar<T>::set;
  using PolyVar<T>::get;
  using PolyVar<T>::bound;
  using PolyVar<T>::bndord;
  using PolyVar<T>::_center;

private:
  //! @brief Pointer to Chebyshev model environment
  CModel<T> *_CM;

  //! @brief Pointer to internal Chebyshev variable in Chebyshev model environment
  CVar<T>* _CV() const
    { return _CM->_CV; };

  //! @brief Indices of terms from product of two terms <tt>imon</tt> and <tt>jmon</tt> (size 2^nvar())
  unsigned int* _prodmon
    ( const unsigned imon, const unsigned jmon ) const
    { return _CM->_prodmon[imon][jmon]; };

  //! @brief Bound on Chebyshev monomial term <tt>imon</tt>
  const T& _bndmon
    ( const unsigned imon ) const
    { return _CM->_bndmon[imon]; };

  //! @brief Bound on Chebyshev basis functions
  const T*const* _bndpow
    () const
    { return _CM->_bndpow; };

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

  //! @brief Array of Chebyshev interpolant coefficients
  double* _coefinterp() const
    { return _CM->_resize_coefinterp(); };

  //! @brief Product of multivariate Chebyshev polynomials in sparse format
  void _sprod
    ( const CVar<T>&CV1, const CVar<T>&CV2, double*coefmon,
      std::set<unsigned>&ndxmon, double&coefrem ) const
    { return _CM->_sprod( CV1, CV2, coefmon, ndxmon, coefrem ); }

  //! @brief Squaring of multivariate Chebyshev polynomials in sparse format
  void _ssqr
    ( const CVar<T>&CV, double*coefmon,
      std::set<unsigned>&ndxmon, double&coefrem ) const
    { return _CM->_ssqr( CV, coefmon, ndxmon, coefrem ); }

public:
  /** @addtogroup CHEBYSHEV Chebyshev Model Arithmetic for Factorable Functions
   *  @{
   */
  //! @brief Get pointer to linked Chebyshev model environment
  CModel<T>* env() const
    { return _CM; }

  //! @brief Constructor of Chebyshev variable for a real scalar
  CVar
    ( const double d=0. );

  //! @brief Constructor of Chebyshev variable for a remainder bound
  CVar
    ( const T&B );

  //! @brief Constructor of Chebyshev variable with index <a>ix</a> (starting from 0) and bounded by <a>X</a>
  CVar
    ( CModel<T>*CM, const unsigned ix, const T&X );

  //! @brief Copy constructor of Chebyshev variable in different Chebyshev model environment (with implicit type conversion)
  template <typename U> CVar
    ( CModel<T>*&CM, const CVar<U>&CV );

  //! @brief Copy constructor of Chebyshev variable in different Chebyshev model environment (with explicit type conversion as given by class member function <a>method</a>)
  template <typename U> CVar
    ( CModel<T>*&CM, const CVar<U>&CV, const T& (U::*method)() const );
  //! @brief Copy constructor of Chebyshev variable in different Chebyshev model environment (with explicit type conversion as given by non-class member function <a>method</a>)
  template <typename U> CVar
    ( CModel<T>*&CM, const CVar<U>&CV, T (*method)( const U& ) );

  //! @brief Copy constructor of Chebyshev variable
  CVar
    ( const CVar<T>&CV )
    : PolyVar<T>( CV ), _CM( CV._CM )
    {
#ifdef  MC__CMODEL_CHECK_PMODEL
      if( _CM != dynamic_cast< CModel<T>* >( PolyVar<T>::_CM ) ) assert( false );
#endif
    }

  //! @brief Destructor of Chebyshev variable
  ~CVar()
    {}
  /** @} */

private:
  //! @brief Private constructor for real scalar in Chebyshev model environment <tt>CM</tt>
  CVar
    ( CModel<T>*CM, const double d=0. );

  //! @brief Private constructor for remainder bound in Chebyshev model environment <tt>CM</tt>
  CVar
    ( CModel<T>*CM, const T&B );

  //! @brief Set Chebyshev variable with index <a>ix</a> (starting from 0) and bounded by <a>X</a>
  CVar<T>& _set
    ( const unsigned ivar, const T&X );

public:
  /** @addtogroup CHEBYSHEV Chebyshev Model Arithmetic for Factorable Functions
   *  @{
   */
  //! @brief Set Chebyshev variable with index <a>ix</a> (starting from 0) and bounded by <a>X</a>
  CVar<T>& set
    ( CModel<T>*CM, const unsigned ix, const T&X )
    { set( CM ); _set( ix, X ); return *this; }

  //! @brief Set Chebyshev model environment in Chebyshev variable to <tt>CM</tt>
  CVar<T>& set
    ( CModel<T>*CM, const bool reset=false )
    { if( CM != _CM ){ _CM = CM; PolyVar<T>::set( _CM, reset ); } return *this; }

  //! @brief Retreive bound on variable using default bounder in U arithmetic
  template <typename U> U bound
    ( const U*const*bndbasis, const U&bndrem ) const
    { return _polybound( bndbasis ) + bndrem; }

  //! @brief Retreive bound on variable using bounder <a>type</a> in U arithmetic
  template <typename U> U bound
    ( const U*const*bndbasis, const U&bndrem, const int type ) const
    { return _polybound( bndbasis, type ) + bndrem; }

  //! @brief Evaluate polynomial part at <tt>x</tt>
  double polynomial
    ( const double*x ) const;

  //! @brief Shortcut to mc::CVar::polynomial
  double P
    ( const double*x ) const
    { return polynomial( x ); }

  //! @brief Return new Chebyshev variable corresponding to the derivative model with respect to variable <a>i</a> and a zero remainder
// dTn/dx = 2*n* sum'(k=0,...,n-1 w/ n-p-k even:  
  CVar<T> polydiff
    ( const unsigned i )
    const
    { CVar<T> var = *this; *(var._bndrem) = 0.; return var; }

  //! @brief Return new Chebyshev variable with same multivariate polynomial part but zero remainder
  CVar<T> polynomial
    ()
    const
    { CVar<T> var = *this; *(var._bndrem) = 0.; return var; }

  //! @brief Shortcut to mc::CVar::polynomial
  CVar<T> P
    ()
    const
    { return polynomial(); }

  //! @brief Center remainder term of Chebyshev variable
  CVar<T>& center
    ()
    { _center(); return *this; }

  //! @brief Shortcut to mc::CVar::center
  CVar<T>& C
    ()
    { return center(); }

  //! @brief Get coefficient of constant term in Chebyshev variable. The value of this coefficient is reset to 0 if <tt>reset=true</tt>, otherwise it is left unmodified (default).
  double constant
    ( const bool reset=false );

  //! @brief Get pointer to array of size <tt>nvar</tt> with coefficients of linear term in Chebyshev variable. Optional argument <tt>X</tt> is the actual variable range, e.g. in case some scaling has been applied (see mc::CVar::scale)
  double* linear
    () const;
    //( const T*X=0 ) const;

  //! @brief Get coefficients of linear term for variable <tt>ivar</tt> in Chebyshev variable. The value of this coefficient is reset to 0 if <tt>reset=true</tt>, otherwise it is left unmodified (default).
  double linear
    ( const unsigned ivar, const bool reset=false );

  //! @brief Get coefficients of linear term for variable <tt>ivar</tt> in Chebyshev variable. The value of this coefficient is reset to 0 if <tt>reset=true</tt>, otherwise it is left unmodified (default). Optional argument <tt>Xvar</tt> is the actual range for variable <tt>ivar</tt>, e.g. in case some scaling has been applied (see mc::CVar::scale)
  //double linear
  //  ( const unsigned ivar, const T&Xvar, const bool reset );

  //! @brief Scale coefficients in Chebyshev variable for the reduced variable <a>X</a>
  CVar<T> scale
    ( const T*X ) const;
 /** @} */

  CVar<T>& operator =
    ( const CVar<T>& );
  CVar<T>& operator =
    ( const double );
  CVar<T>& operator =
    ( const T& );
  template <typename U> CVar<T>& operator +=
    ( const CVar<U>& );
  template <typename U> CVar<T>& operator +=
    ( const U& );
  CVar<T>& operator +=
    ( const double );
  template <typename U> CVar<T>& operator -=
    ( const CVar<U>& );
  template <typename U> CVar<T>& operator -=
    ( const U& );
  CVar<T>& operator -=
    ( const double );
  CVar<T>& operator *=
    ( const CVar<T>& );
  CVar<T>& operator *=
    ( const double );
  CVar<T>& operator *=
    ( const T& );
  CVar<T>& operator /=
    ( const CVar<T>& );
  CVar<T>& operator /=
    ( const double );

private:
  //! @brief Update bounds for all terms of degrees <tt>iord=0,...,_nord</tt> in <tt>_bndord</tt>
  void _update_bndord() const;

  //! @brief Polynomial range bounder using specified bounder <a>type</a> with basis functions <a>bndbasis</a> in U arithmetic
  template <typename U> U _polybound
    ( const U*const*bndbasis, const int type ) const
    { return ( _CM? _CM->_polybound( _coefmon, bndbasis, type, _ndxmon ): _coefmon[0] ); }

  //! @brief Polynomial range bounder using default bounder in U arithmetic
  template <typename U> U _polybound
    ( const U*const*bndbasis ) const
    { return _polybound( bndbasis, _CM? _CM->options.BOUNDER_TYPE: 0 ); }

  //! @brief Polynomial range bounder using specified bounder <a>type</a>
  T _polybound
    ( const int type ) const;

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
    { CModel<T>::_interpolation( coefmon, nord()+_CM->options.INTERP_EXTRA, bound(), f ); }

  //! @brief Apply Chebyshev composition to variable <a>CVI</a> using the coefficients <a>coefmon</a> of the outer function
  template <typename U> CVar<T> _composition
    ( const U* coefouter ) const
    { return CModel<T>::_composition( coefouter, nord(), *this ); }

  //! @brief Scale current variable in order for its range to be within [-1,1], with <a>c</a> and <a>w</a> respectively the center and width, respectively, of the orginal variable range
  CVar<T> _rescale
    ( const double w, const double c ) const
    { return( !isequal(w,0.)? (*this-c)/w: c ); }

  //! @brief Return an array of Chebyshev variables representing the coefficients in the univariate polynomial for variable <a>ivar</a> only
  CVar<T>* _single
    ( const unsigned ivar );
};

////////////////////////////////// CModel //////////////////////////////////////

template <typename T> inline void
CModel<T>::_size
()
{
#ifndef MC__CVAR_SPARSE_PRODUCT_NAIVE
  if( !_sparse ) _set_prodmon();
  else           _prodmon = 0;
#else
  _set_prodmon();
#endif
  _bndpow = new T*[_nvar];
  for( unsigned i=0; i<_nvar; i++ ) _bndpow[i] = 0;
  _bndmon = new T[_nmon];  
  _bndvar = new T[_nvar];
  for( unsigned i=0; i<_nvar; i++ ) _bndvar[i] = 0.;
  _ncoefinterp = 0; _coefinterp = 0;

  _CV = new CVar<T>( this );
}

template <typename T> inline CModel<T>*
CModel<T>::lift
( const unsigned nx, const T*X ) const
{
  CModel<T>* CM  = new CModel<T>( _nvar+nx, _nord );
  unsigned ix = 0;
  for( ; ix<_nvar; ix++ )
    CVar<T> var( CM, ix, _bndvar[ix] );
  for( ; ix<nx; ix++ )
    CVar<T> var( CM, ix, X[ix-_nvar] );
  return CM;
}

template <typename T> inline void
CModel<T>::_reset()
{
  for( unsigned i=0; i<_nvar; i++ ){
    delete[] _bndpow[i];
    _bndpow[i] = 0;
  }
}

template <typename T> inline void
CModel<T>::_cleanup()
{
  for( unsigned i=0; _prodmon && i<_nmon; i++ ){
    for( unsigned j=0; j<=i; j++ )
      delete[] _prodmon[i][j];
    delete[] _prodmon[i];
  }
  delete[] _prodmon;
  for( unsigned i=0; i<_nvar; i++ )
    delete[] _bndpow[i];
  delete[] _bndpow;
  delete[] _bndmon;
  delete[] _bndvar;
  delete[] _coefinterp;
  delete _CV;
}

template <typename T> inline double*
CModel<T>::_resize_coefinterp()
{
  if( _ncoefinterp > _nord+options.INTERP_EXTRA ) return _coefinterp;
  delete[] _coefinterp;
  _ncoefinterp = _nord+options.INTERP_EXTRA+1;
  _coefinterp = new double[_ncoefinterp];
  return _coefinterp;
}

template <typename T> inline void
CModel<T>::_set_prodmon()
{
  _prodmon = new unsigned int**[_nmon];
  size_t prodmon_max = _nvar+1;
  for( unsigned i=0; i<_nvar && i<_nord; i++ ){  // <-- SIZE COULD BE TOO RESTRICTIVE!!!
    if( prodmon_max > std::numeric_limits<size_t>::max()/2 )
      throw typename PolyModel::Exceptions( PolyModel::Exceptions::MAXSIZE );
    prodmon_max *= 2;
  }
  unsigned *iexp = new unsigned int[prodmon_max];
  size_t prodmon_tot = 0;

  // Loop over all monic terms
  for( unsigned iord=0; iord<=_nord; iord++ ){    
    for( unsigned imon=_posord[iord]; imon<_posord[iord+1]; imon++ ){

      // Loop over all monics whose indices are lower than or equal to imon
      _prodmon[imon] = new unsigned int*[imon+1];
      for( unsigned jord=0, jmon=0; jord<=iord; jord++ ){    
        for( ; jmon<_posord[jord+1] && jmon<=imon; jmon++ ){

          // Current monic operand
          iexp[0] = 0; // used to store partial order
          for( unsigned ivar=0; ivar<_nvar; ivar++ )
            iexp[ivar+1] = _expmon[imon*_nvar+ivar];
#ifdef MC__CMODEL_DEBUG
          mc::display( 1, _nvar, iexp+1, 1, "iexp:", std::cout );
          mc::display( 1, _nvar, _expmon+jmon*_nvar, 1, "jexp:", std::cout );
#endif
          // Construct product monics recursively and only keep those of order <= _nord
          unsigned nprodmax = 1, nprodmon = 1, kvar = 0;
          for( unsigned ivar=0; ivar<_nvar; ivar++ )
            if( iexp[ivar+1] && _expmon[jmon*_nvar+ivar] ) nprodmax*=2;
          _set_prodmon_exp( kvar, _expmon+jmon*_nvar, jord, iexp, nprodmon );
          _prodmon[imon][jmon] = new unsigned int[nprodmon+2];
          _prodmon[imon][jmon][0] = nprodmax;
          _prodmon[imon][jmon][1] = 0;
          for( unsigned kmon=0; kmon<nprodmon; kmon++ ){
#ifdef MC__CMODEL_DEBUG
            mc::display( 1, _nvar+1, iexp+kmon*(_nvar+1), 1, "ijexp:", std::cout );
#endif
            // Eliminate monic terms with order > _nord
            if( iexp[kmon*(_nvar+1)] > _nord ) continue;
            _prodmon[imon][jmon][++_prodmon[imon][jmon][1]+1] = _loc_expmon( iexp+kmon*(_nvar+1)+1 );
          }
          prodmon_tot += (imon==jmon?_prodmon[imon][jmon][1]:2*_prodmon[imon][jmon][1]);
#ifdef MC__CMODEL_DEBUG
          std::ostringstream oheadi;
          oheadi << "_prodmon[" << imon << "," << jmon << "]:  max=" << nprodmax
                 << "  cur=" << _prodmon[imon][jmon][1];
          mc::display( 1, _prodmon[imon][jmon][1], _prodmon[imon][jmon]+2, 1, oheadi.str(), std::cout );
#endif
        }
      }
    }
  }
#ifdef MC__CMODEL_DEBUG
  std::ostringstream oheadi;
  std::cerr << "  " << _nvar << "  " << _nord << "  " << prodmon_tot << std::endl;
#endif

  delete[] iexp;
}

template <typename T> inline void
CModel<T>::_set_prodmon_exp
( unsigned int&kvar, const unsigned int*iexp, const unsigned iord,
  unsigned int*iprodexp, unsigned int&nprodmon )
{
  if( kvar == _nvar || !nprodmon ) return;

  int nprodnew = 0;
  for( unsigned imon=0; imon<nprodmon; imon++ ){
    // Product with Chebyshev basis function T0
    if( !iexp[kvar] || !iprodexp[imon*(_nvar+1)+kvar+1] ){
      iprodexp[imon*(_nvar+1)] += iprodexp[imon*(_nvar+1)+kvar+1] + iexp[kvar];
      if( iprodexp[imon*(_nvar+1)] <= _nord )
        iprodexp[imon*(_nvar+1)+kvar+1] += iexp[kvar];
    }
    // Product with Chebyshev basis function Ti, i>0
    else{
      // Only add term if resulting monic index remain less than or equal to _nord
      if( iprodexp[imon*(_nvar+1)] + iprodexp[imon*(_nvar+1)+kvar+1] + iexp[kvar] <= _nord ){
        for( unsigned ivar=0; ivar<=_nvar; ivar++ )
          iprodexp[(nprodmon+nprodnew)*(_nvar+1)+ivar] = iprodexp[imon*(_nvar+1)+ivar];
        iprodexp[(nprodmon+nprodnew)*(_nvar+1)] += iprodexp[imon*(_nvar+1)+kvar+1] + iexp[kvar];
        iprodexp[(nprodmon+nprodnew)*(_nvar+1)+kvar+1] += iexp[kvar];
        nprodnew++;
      }
      if( iexp[kvar] <= iprodexp[imon*(_nvar+1)+kvar+1] ){
        iprodexp[imon*(_nvar+1)] += iprodexp[imon*(_nvar+1)+kvar+1] - iexp[kvar];
        if( iprodexp[imon*(_nvar+1)] <= _nord )
          iprodexp[imon*(_nvar+1)+kvar+1] -= iexp[kvar];
      }
      else{
        iprodexp[imon*(_nvar+1)] += iexp[kvar] - iprodexp[imon*(_nvar+1)+kvar+1];      
        if( iprodexp[imon*(_nvar+1)] <= _nord )
          iprodexp[imon*(_nvar+1)+kvar+1] = iexp[kvar] - iprodexp[imon*(_nvar+1)+kvar+1];
      }
    }
  }
  nprodmon += nprodnew;
  return _set_prodmon_exp( ++kvar, iexp, iord, iprodexp, nprodmon );
}

template <typename T> inline void
CModel<T>::_set_bndmon()
{
  if( !_modvar ) return;
  
  _bndmon[0] = 1.;
  for( unsigned i=1; i<_nmon; i++ ){
    _bndmon[i] = 1.;
    for( unsigned j=0; j<_nvar; j++)
      if( _bndpow[j] ) _bndmon[i] *= _bndpow[j][_expmon[i*_nvar+j]];
  }
  _modvar = false;

#ifdef MC__CMODEL_DEBUG
  mc::display( 1, _nmon, _bndmon, 1, "_bndmon", std::cout );
#endif
}

template <typename T> inline void
CModel<T>::_set_bndpow
( const unsigned i, const T&X, const double ref, const double scal )
{
  if( i>=_nvar ) throw Exceptions( Exceptions::INIT );

  delete[] _bndpow[i];
  _bndpow[i]  = _get_bndpow( _nord, X, ref, scal );
  _bndvar[i]  = X;
  _scalvar[i] = scal;
  _refvar[i]  = ref;
  _modvar = true;

#ifdef MC__CMODEL_DEBUG
  mc::display( 1, _nvar, _bndvar, 1, "_bndvar", std::cout );
#endif
}

template <typename T> template <typename U> inline U*
CModel<T>::_get_bndpow
( const unsigned nord, const U&X, const double ref, const double scal )
{
  U *Xrcheb = new U[nord+1];
  Xrcheb[0] = 1.;
  if( nord ) Xrcheb[1] = ( X - ref ) / scal;
  for( unsigned i=2; i<=nord; i++ )
    Xrcheb[i] = Op<U>::cheb( Xrcheb[1], i );
 return Xrcheb;
}

template <typename T> template <typename U> inline U*
CModel<T>::_get_bndpow
( const unsigned nord, const U&X )
{
  U *Xrcheb = new U[nord+1];
  Xrcheb[0] = 1.;
  if( nord ) Xrcheb[1] = X;
  for( unsigned i=2; i<=nord; i++ )
    Xrcheb[i] = Op<U>::cheb( Xrcheb[1], i );
 return Xrcheb;
}

template <typename T> template <typename U> inline U**
CModel<T>::get_basis
( const unsigned nord, const U*Xvar, const bool scaled ) const
{
  assert ( _bndvar );
  U** Xrcheb = new U*[_nvar];
  for( unsigned i=0; i<_nvar; i++ )
    Xrcheb[i] = scaled? _get_bndpow( nord, Xvar[i] ):
                        _get_bndpow( nord, Xvar[i], _refvar[i], _scalvar[i] );
  return Xrcheb;
}
/*
template <typename T> template <typename U> inline std::pair<unsigned,U*>
CModel<T>::get_bndmon
( const unsigned nord, const U*Xvar, const bool scaled ) const
{
  const unsigned maxord = nord>_nord? _nord: nord;
  U**basis = get_basis( maxord, Xvar, scaled );
  const unsigned nmon = _posord[maxord+1];
  U* bndmon = new U[nmon];
  bndmon[0] = 1.;
  for( unsigned i=1; i<nmon; i++ ){
    bool first = true;
    for( unsigned j=0; j<_nvar; j++){
      if( !basis[j] ) continue;
      if( first ){
        bndmon[i] = basis[j][_expmon[i*_nvar+j]];
        first = false;
      }
      else
        bndmon[i] *= basis[j][_expmon[i*_nvar+j]];
    }
  }
  for( unsigned j=0; j<_nvar; j++) delete[] basis[j];
  delete[] basis;

#ifdef MC__CMODEL_DEBUG
  mc::display( 1, nmon, bndmon, 1, "bndmon", std::cout );
#endif
  return std::make_pair(nmon,bndmon);
}
*/
template <typename T> template <typename U> inline void
CModel<T>::get_bndmon
( const unsigned nord, U*bndmon, const U*Xvar, const bool scaled ) const
{
  if( !bndmon ) return;
  const unsigned maxord = nord>_nord? _nord: nord;
  U**basis = get_basis( maxord, Xvar, scaled );
  const unsigned nmon = _posord[maxord+1];
  bndmon[0] = 1.;
  for( unsigned i=1; i<nmon; i++ ){
    bool first = true;
    for( unsigned j=0; j<_nvar; j++){
      if( !basis[j] ) continue;
      if( first ){
        bndmon[i] = basis[j][_expmon[i*_nvar+j]];
        first = false;
      }
      else
        bndmon[i] *= basis[j][_expmon[i*_nvar+j]];
    }
  }
  for( unsigned j=0; j<_nvar; j++) delete[] basis[j];
  delete[] basis;

#ifdef MC__CMODEL_DEBUG
  mc::display( 1, nmon, bndmon, 1, "bndmon", std::cout );
#endif
  return;
}

template <typename T> template <typename U> inline void
CModel<T>::get_bndmon
( const unsigned nord, U*bndmon, const std::set<unsigned>&ndxmon,
  const U*Xvar, const bool scaled ) const
{
  if( !bndmon || ndxmon.empty() ) return;
  const unsigned maxord = nord>_nord? _nord: nord;
  U**basis = get_basis( maxord, Xvar, scaled );
  std::set<unsigned>::const_iterator it = ndxmon.begin();
  if( !*it ){ bndmon[0] = 1.; ++it; }
  for( ; it != ndxmon.end() && *it < _posord[maxord+1]; ++it ){
    bool first = true;
    for( unsigned j=0; j<_nvar; j++){
      if( !basis[j] ) continue;
      if( first ){
        bndmon[*it] = basis[j][_expmon[(*it)*_nvar+j]];
        first = false;
      }
      else
        bndmon[*it] *= basis[j][_expmon[(*it)*_nvar+j]];
    }
  }
  for( unsigned j=0; j<_nvar; j++) delete[] basis[j];
  delete[] basis;

#ifdef MC__CMODEL_DEBUG
  mc::display( 1, nmon, bndmon, 1, "bndmon", std::cout );
#endif
  return;
}

template <typename T> inline CVar<T>
CModel<T>::_intpow
( const CVar<T>&CV, const int n ) const
{
  if( n == 0 ) return 1.;
  else if( n == 1 ) return CV;
  return n%2 ? sqr( _intpow( CV, n/2 ) ) * CV : sqr( _intpow( CV, n/2 ) );
}

template <typename T> inline void
CModel<T>::_interpolation
( double*coefmon, const unsigned nord, const T&X, puniv f )
{
  double b( Op<T>::mid(X) ), a( Op<T>::u(X)-b ), x[nord+1], fx[nord+1];
  double mulconst( PI/(2.*double(nord+1)) );
  for( unsigned i(0); i<=nord; i++ ){
    x[i]  = std::cos(mulconst*(2.*double(i)+1.));
    fx[i] = f( a*x[i]+b );
  }

  switch( nord ){
  case 0:
    coefmon[0] = fx[0];
    return;
  case 1:
    coefmon[0] = 0.5 * ( fx[0] + fx[1] );
    coefmon[1] = ( fx[1] - fx[0] ) / ( x[1] - x[0] );
    return;
  default:
    for( unsigned i(0); i<=nord; i++ ){
      double mulconst2( std::cos(mulconst*double(i)) ),
             mulconst3( 4*std::pow(mulconst2,2)-2 ), 
             b0( 0 ), b1( 0 );
      b0 = fx[nord];
      b1 = fx[nord-1] + mulconst3*b0;
      for( unsigned j=nord-2; j>1; j-=2 ){
        b0 = fx[j] + mulconst3*b1 - b0;
        b1 = fx[j-1] + mulconst3*b0 - b1;
      }
      if( !(nord%2) )
        b0 = fx[0] + mulconst3*b1 - b0 - b1;
      else{
        b0 = fx[1] + mulconst3*b1 - b0;
        b0 = fx[0] + mulconst3*b0 - b1 - b0;
      }
      coefmon[i] = 2./double(nord+1)*mulconst2*b0;
    }
    coefmon[0] *=0.5;
    return;
  }
}

template <typename T> template <typename U> inline CVar<T>
CModel<T>::_composition
( const U* coefouter, const unsigned nord, const CVar<T>& CVinner )
{
  //composition based on http://en.wikipedia.org/wiki/Clenshaw_algorithm#Special_case_for_Chebyshev_series
  if( !nord )
    return coefouter[0];

  else if( nord == 1 )
    return CVinner * coefouter[1] + coefouter[0];

  CVar<T> CVinnerx2 = 2. * CVinner;
//std::cout << "CVinner:" << CVinner;
//std::cout << "CVinnerx2:" << CVinnerx2;
  CVar<T> CV1 = coefouter[nord];
//std::cout << "CV1:" << CV1;
//for( unsigned i=0; i<CV1.nmon(); i++ ) std::cout << CV1._coefmon[i] << std::endl;
  CVar<T> CV2 = coefouter[nord-1] + CVinnerx2 * CV1;
//std::cout << "CV2:" << CV2;
//for( unsigned i=0; i<CV2.nmon(); i++ ) std::cout << CV2._coefmon[i] << std::endl;
  for( unsigned i=nord-2; i>1; i-=2 ){
    CV1 = coefouter[i]   + CVinnerx2 * CV2 - CV1;
//std::cout << "CV1:" << CV1;
    CV2 = coefouter[i-1] + CVinnerx2 * CV1 - CV2;
//std::cout << "CV2:" << CV2;
  }
  if( !(nord%2) )
    return coefouter[0] + CVinner * CV2 - CV1;
//CVar<T> tmp = coefouter[1] + CVinnerx2 * CV2 - CV1;
//std::cout << "coefouter[1] + CVinnerx2 * CV2 - CV1:" << tmp;
//for( unsigned i=0; i<tmp.nmon(); i++ ) std::cout << tmp._coefmon[i] << std::endl;
  CV1 = coefouter[1] + CVinnerx2 * CV2 - CV1;
//for( unsigned i=0; i<CV1.nmon(); i++ ) std::cout << CV1._coefmon[i] << std::endl;
//std::cout << "CV1:" << CV1;
//std::cout << "CVinner * CV1:" << CVinner * CV1;
  return coefouter[0] + CVinner * CV1 - CV2;
}

template <typename T> inline void
CModel<T>::_sscal1D
( const std::map<unsigned,double>&CVmap, const double&coefscal,
  std::map<unsigned,double>&CVRmap )
const
{
  if( coefscal == 0. ) return;
  CVRmap = CVmap;
  if( coefscal == 1. ) return;
  for( auto it=CVRmap.begin(); it!=CVRmap.end(); ++it )
    it->second *= coefscal;
  return;
}

template <typename T> inline void
CModel<T>::_slift1D
( const std::map<unsigned,double>&CVmap, const double dscal,
  std::map<unsigned,double>&CVRmap )
const
{
  for( auto it=CVmap.begin(); it!=CVmap.end(); ++it ){
    auto pmon = CVRmap.insert( *it );
    if( pmon.second ){ if( dscal != 1. ) pmon.first->second *= dscal; }
    else{              pmon.first->second += dscal==1.? it->second: it->second * dscal; }
  }
}

template <typename T> inline void
CModel<T>::_slift1D
( const std::map<unsigned,double>&CVmap, const double dscal,
  std::map<unsigned,double>&CVRmap, double&coefrem,
  const unsigned ndxvar, const unsigned ndxord, unsigned*iexp )
const
{
  for( auto it=CVmap.begin(); it!=CVmap.end(); ++it ){
    unsigned nexp = iexp[ndxvar] = ndxord; // initialize total index
    for( unsigned ivar=ndxvar+1; ivar<_nvar; ivar++ )
      nexp += iexp[ivar] = expmon(it->first)[ivar]; // build total index
    if( nexp > _nord ){ // append to remainder coefficient if total order too large
      coefrem += std::fabs( dscal==1.? it->second: it->second * dscal );
      continue;
    }
    auto pmon = CVRmap.insert( std::make_pair( _loc_expmon(iexp), dscal==1.? it->second: it->second * dscal ) );
    if( !pmon.second ) pmon.first->second += dscal==1.? it->second: it->second * dscal;
  }
}

template <typename T> inline void
CModel<T>::_slift1D
( const std::map<unsigned,double>&CVmap, double&coefrem )
const
{
  for( auto it=CVmap.begin(); it!=CVmap.end(); ++it )
    coefrem += std::fabs( it->second );
}

template <typename T> inline void
CModel<T>::_sdisp1D
( const std::map<unsigned,double>&CVmap,
  const unsigned ndxvar, const std::string&CVname, std::ostream&os )
const
{
  os << CVname;
  for( auto it=CVmap.begin(); it!=CVmap.end(); ++it ){
    if( it != CVmap.begin() ) os << " + ";
    os << it->second;
    for( unsigned ivar=ndxvar; ivar<_nvar; ivar++ ) 
      if( expmon(it->first)[ivar] ) os << "·T" << expmon(it->first)[ivar] << "(X" << ivar << ")";
  }
  os << std::endl;
}

template <typename T> inline void
CModel<T>::_sdisp1D
( const std::vector<std::map<unsigned,double>>&CVmap,
  const unsigned ndxvar, const std::string&CVname, std::ostream&os )
const
{
  os << CVname;
  for( unsigned i=0; i<=_nord; i++ ){
    if( i ) os << " + T" << i << "(X" << ndxvar << ") ·";
    os << " [ ";
    for( auto it=CVmap[i].begin(); it!=CVmap[i].end(); ++it ){
      if( it != CVmap[i].begin() ) os << " + ";
      os << it->second;
      for( unsigned ivar=ndxvar+1; ivar<_nvar; ivar++ ) 
        if( expmon(it->first)[ivar] ) os << "·T" << expmon(it->first)[ivar] << "(X" << ivar << ")";
    }
    os << " ]";
  }
  os << std::endl;
}

template <typename T> inline void
CModel<T>::_sprod1D
( const std::vector<std::map<unsigned,double>>&CV1map,
  const std::vector<std::map<unsigned,double>>&CV2map,
  std::map<unsigned,double>&CVRmap, double&coefrem,
  const unsigned ndxvar )
const
{
  std::vector<unsigned> vexp( _nvar, 0 ); unsigned* iexp = vexp.data();

  // construct product matrix of polynomial coefficients
  std::vector<std::map<unsigned,double>> CV12map( (_nord+1)*(_nord+1) );
  for( unsigned iord1=0; iord1<=_nord; iord1++ ){
    if( CV1map[iord1].empty() ) continue; // no term
    if( CV1map[iord1].size() == 1 && !CV1map[iord1].begin()->first ){ // constant term only
      for( unsigned iord2=0; iord2<=_nord; iord2++ )
        _sscal1D( CV2map[iord2], CV1map[iord1].begin()->second, CV12map[iord1*(_nord+1)+iord2] );
      continue;
    }
    for( unsigned iord2=0; iord2<=_nord; iord2++ ){ // general polynomial
      if( CV2map[iord2].empty() ) continue; // no term
      if( CV2map[iord2].size() == 1 && !CV2map[iord2].begin()->first ){ // constant term only
        _sscal1D( CV1map[iord1], CV2map[iord2].begin()->second, CV12map[iord1*(_nord+1)+iord2] );
        continue;
      }
#ifdef MC__POLYMODEL_DEBUG_SPROD
      std::cout << "Term (" << iord1 << "," << iord2 << "):\n";
#endif
      std::vector<std::map<unsigned,double>> CV11map( _nord+1 ), CV22map( _nord+1 ); // component maps for both coefficients
      for( auto it=CV1map[iord1].begin(); it!=CV1map[iord1].end(); ++it ){
        for( unsigned ivar=ndxvar+2; ivar<_nvar; ivar++ ) iexp[ivar] = expmon(it->first)[ivar];
        CV11map[ expmon(it->first)[ndxvar+1] ].insert( std::make_pair( _loc_expmon(iexp), it->second ) );
      }
#ifdef MC__POLYMODEL_DEBUG_SPROD
      _sdisp1D( CV11map, ndxvar+1, "Var #1: " );
#endif
      for( auto it=CV2map[iord2].begin(); it!=CV2map[iord2].end(); ++it ){
        for( unsigned ivar=ndxvar+2; ivar<_nvar; ivar++ ) iexp[ivar] = expmon(it->first)[ivar];
        CV22map[ expmon(it->first)[ndxvar+1] ].insert( std::make_pair( _loc_expmon(iexp), it->second ) );
      }
#ifdef MC__POLYMODEL_DEBUG_SPROD
      _sdisp1D( CV22map, ndxvar+1, "Var #2: " );
#endif
      _sprod1D( CV11map, CV22map, CV12map[iord1*(_nord+1)+iord2], coefrem, ndxvar+1 );
    }
  }

  // construct result map and augment remainder as appropriate
  CVRmap.clear();
  for( unsigned l=0; l<=_nord; l++ )
    _slift1D( CV12map[l*(_nord+1)+l], l?1.:2., CVRmap );
  for( unsigned k=1; k<=_nord; k++ ){
    for( unsigned l=0; l<=k; l++ )
      _slift1D( CV12map[(k-l)*(_nord+1)+l], l&&k-l?1.:2., CVRmap, coefrem, ndxvar, k, iexp );
    for( unsigned l=1; l+k<=_nord; l++ ){
      _slift1D( CV12map[l*(_nord+1)+l+k],   1., CVRmap, coefrem, ndxvar, k, iexp );
      _slift1D( CV12map[(l+k)*(_nord+1)+l], 1., CVRmap, coefrem, ndxvar, k, iexp );
    }
  }
  for( unsigned k=_nord+1; k<=2*_nord; k++ )
    for( unsigned l=k-_nord; l<=_nord; l++ )
      _slift1D( CV12map[(k-l)*(_nord+1)+l], coefrem );
  // rescale the coefficients by 0.5
  for( auto it=CVRmap.begin(); it!=CVRmap.end(); ++it ) it->second *= 0.5;
#ifdef MC__POLYMODEL_DEBUG_SPROD
  _sdisp1D( CVRmap, ndxvar, "Prod: " );
#endif
}

template <typename T> inline void
CModel<T>::_sprod
( const CVar<T>&CV1, const CVar<T>&CV2, double*coefmon,
  std::set<unsigned>&ndxmon, double&coefrem )
const
{
  // Construct vectors of coefficients for variable #0
  std::vector<unsigned> vexp( _nvar, 0 ); unsigned* iexp = vexp.data();
  std::vector<std::map<unsigned,double>> CV1map( _nord+1 ), CV2map( _nord+1 );
  for( auto it=CV1._ndxmon.begin(); it!=CV1._ndxmon.end(); ++it ){
    for( unsigned ivar=1; ivar<_nvar; ivar++ ) iexp[ivar] = expmon(*it)[ivar];
    CV1map[ expmon(*it)[0] ].insert( std::make_pair( _loc_expmon(iexp), CV1._coefmon[*it] ) );
  }
#ifdef MC__POLYMODEL_DEBUG_SPROD
  _sdisp1D( CV1map, 0, "Var #1: " );
#endif
  for( auto it=CV2._ndxmon.begin(); it!=CV2._ndxmon.end(); ++it ){
    for( unsigned ivar=1; ivar<_nvar; ivar++ ) iexp[ivar] = expmon(*it)[ivar];
    CV2map[ expmon(*it)[0] ].insert( std::make_pair( _loc_expmon(iexp), CV2._coefmon[*it] ) );
  }
#ifdef MC__POLYMODEL_DEBUG_SPROD
  _sdisp1D( CV2map, 0, "Var #2: " );
#endif

  // Initialize recursive product of univariate Chebyshev polynomials
  std::map<unsigned,double> CVRmap;
  _sprod1D( CV1map, CV2map, CVRmap, coefrem, 0 );

  // Populate index set, coefficient values and higher-order term bound
  for( auto it=CVRmap.begin(); it!=CVRmap.end(); ++it )
    coefmon[*(ndxmon.insert( it->first ).first)] = it->second;
  coefrem *= 0.5;
}

template <typename T> inline void
CModel<T>::_ssqr
( const CVar<T>&CV, double*coefmon,
  std::set<unsigned>&ndxmon, double&coefrem )
const
{
  // Construct vectors of coefficients for variable #0
  std::vector<unsigned> vexp( _nvar, 0 ); unsigned* iexp = vexp.data();
  std::vector<std::map<unsigned,double>> CVmap( _nord+1 );
  for( auto it=CV._ndxmon.begin(); it!=CV._ndxmon.end(); ++it ){
    for( unsigned ivar=1; ivar<_nvar; ivar++ ) iexp[ivar] = expmon(*it)[ivar];
    CVmap[ expmon(*it)[0] ].insert( std::make_pair( _loc_expmon(iexp), CV._coefmon[*it] ) );
  }
#ifdef MC__POLYMODEL_DEBUG_SSQR
  _sdisp1D( CVmap, 0, "Var: " );
#endif

  // Initialize recursive product of univariate Chebyshev polynomials
  std::map<unsigned,double> CVRmap;
  _sprod1D( CVmap, CVmap, CVRmap, coefrem, 0 );

  // Populate index set, coefficient values and higher-order term bound
  for( auto it=CVRmap.begin(); it!=CVRmap.end(); ++it )
    coefmon[*(ndxmon.insert( it->first ).first)] = it->second;
  coefrem *= 0.5;
}

// ==> account for sparse format

template <typename T> template <typename C, typename U> inline U
CModel<T>::_polybound_eigen
( const C*coefmon, const U*bndord, const U*const*bndbasis,
  const std::set<unsigned>&ndxmon ) const
{
  static const double TOL = 1e-8;

  U bndpol = (ndxmon.empty() || ndxmon.find(0)!=ndxmon.end())? coefmon[0]: 0.;
  //U bndpol = coefmon[0];
  if( _nord == 1 ) bndpol += bndord[1];

  else if( _nord > 1 ){
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

  for( unsigned i=3; i<=_nord; i++ ) bndpol += bndord[i];
  return bndpol;
}

// ==> account for sparse format

template <typename T> template <typename C, typename U> inline U
CModel<T>::_polybound_LSB
( const C*coefmon, const U*bndord, const U*const*bndbasis,
  const std::set<unsigned>&ndxmon ) const
{
  static const double TOL = 1e-8;

  U bndpol = (ndxmon.empty() || ndxmon.find(0)!=ndxmon.end())? coefmon[0]: 0.;
  //U bndpol = coefmon[0];
  if( _nord == 1 ) bndpol += bndord[1];

  else if( _nord > 1 ){
    for( unsigned i=_posord[2]; i<_posord[3]; i++ ){
      unsigned k1=_nvar, k2=_nvar;
      const unsigned*iexp = _expmon+i*_nvar;
      for( unsigned j=0; k2==_nvar; j++ ){
        if( iexp[j] == 2 )             k1 = k2 = j;
        else if( iexp[j] && k1 == k2 ) k1 = j;
        else if( iexp[j] )             k2 = j;
      }
      C Ci = (ndxmon.empty() || ndxmon.find(i)!=ndxmon.end())? coefmon[i]: 0.;
      C Ck = (ndxmon.empty() || ndxmon.find(_nvar-k1)!=ndxmon.end())? coefmon[_nvar-k1]: 0.;
      // off-diagonal quadratic terms
      if( k1 < k2 ){
        bndpol += Ci * bndbasis[k1][1] * bndbasis[k2][1];
        continue;
      }
      // linear and diagonal quadratic terms
      const double ai  = Ck, aii = Ci;
      if( std::fabs(aii) > TOL )
        bndpol += (2.*aii)*Op<U>::sqr(bndbasis[k1][1]+ai/(aii*4.))-aii-ai*ai/8./aii;
      else
        bndpol += ai*bndbasis[k1][1] + aii*bndbasis[k1][2];
    }
  }
  // higher-order terms
  for( unsigned i=3; i<=_nord; i++ ) bndpol += bndord[i];
  return bndpol;
}

template <typename T> template <typename C, typename U> inline U
CModel<T>::_polybound_bernstein
( const C*coefmon, const U*bndord, const U*const*bndbasis,
  const std::set<unsigned>&ndxmon )
{
  // Expand binomial coefficient and exponent arrays if needed
#ifdef MC__CVAR_DEBUG_BERNSTEIN
  std::cout << "binom max: " << _binom_size.first << "  "
            << _binom_size.second << std::endl;
#endif
  const unsigned maxord = (options.BOUNDER_ORDER>_nord? 
    options.BOUNDER_ORDER: _nord );
  const poly_size maxmon = std::pow(maxord+1,_nvar);
  _ext_expmon( maxord, true );
  _ext_binom( 2*_nord );
#ifdef MC__CVAR_DEBUG_BERNSTEIN
  std::cout << "binom max: " << _binom_size.first << "  "
            << _binom_size.second << std::endl;
#endif

  // Compute transformation matrix
  double*trmat = new double[(maxord+1)*(_nord+1)];
#ifdef MC__CVAR_DEBUG_BERNSTEIN
  std::cout << "trmat:\n" << std::scientific << std::setprecision(5);
#endif
  for( unsigned j=0, jk=0; j<=maxord; j++ ){
    for( unsigned k=0; k<=_nord; k++, jk++ ){
      trmat[jk] = _transform_bernstein( j, k, maxord );
#ifdef MC__CVAR_DEBUG_BERNSTEIN
      std::cout << "  " << trmat[jk];
#endif
    }
#ifdef MC__CVAR_DEBUG_BERNSTEIN
    std::cout << std::endl;
#endif
  }

  // Compute min/max amongst all Bernstein coefficients
  U bndpol = (ndxmon.empty() || ndxmon.find(0)!=ndxmon.end())? coefmon[0]: 0.;
  //U bndpol = coefmon[0];
#ifdef MC__CVAR_DEBUG_BERNSTEIN
  std::cout << "\n0:  " << bndpol << std::endl;
#endif
  for( poly_size jmon=0; jmon<maxmon; jmon++ ){ // Loop over Bernstein terms
    const unsigned*jexp = _expmon + jmon*_nvar;
    C coefbern = _coef_bernstein( coefmon, jexp, trmat, ndxmon );
    bndpol = Op<U>::hull( bndpol, coefbern );
#ifdef MC__CVAR_DEBUG_BERNSTEIN
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
CModel<T>::_transform_bernstein
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
CModel<T>::_coef_bernstein
( const C*coefmon, const unsigned*jexp, const double*trmat,
  const std::set<unsigned>&ndxmon ) const
{
  // Compute bernstein coefficient with variables indices <tt>jexp</tt>
  C coefbern = (ndxmon.empty() || ndxmon.find(0)!=ndxmon.end())? coefmon[0]: 0.;
#ifdef MC__CVAR_DEBUG_BERNSTEIN
    std::cout << "  [  0]  " << coefmon[0] << "  " << coefbern << std::endl;
#endif
  // Sparse loop over Chebyshev terms
  for( auto it=ndxmon.begin(); it!=ndxmon.end(); ++it ){
    const unsigned*kexp = _expmon + (*it)*_nvar;
    C termbern = coefmon[(*it)]; // Chebyshev coefficient for (*it)
#ifdef MC__CVAR_DEBUG_BERNSTEIN
    std::cout << "  ["; 
    for( unsigned ivar=0; ivar<_nvar; ivar++ )
      std::cout << std::setw(3) << kexp[ivar];
    std::cout << "]";
#endif
    for( unsigned ivar=0; ivar<_nvar; ivar++ )
      termbern *= trmat[jexp[ivar]*(_nord+1)+kexp[ivar]];
    coefbern += termbern;      
#ifdef MC__CVAR_DEBUG_BERNSTEIN
    std::cout << "  " << termbern << "  " << coefbern << std::endl;
#endif
  }  
  // Dense loop over Chebyshev terms
  for( unsigned kmon=1; ndxmon.empty() && kmon<_nmon; kmon++ ){
    const unsigned*kexp = _expmon + kmon*_nvar;
    C termbern = coefmon[kmon]; // Chebyshev coefficient for kmon
#ifdef MC__CVAR_DEBUG_BERNSTEIN
    std::cout << "  ["; 
    for( unsigned ivar=0; ivar<_nvar; ivar++ )
      std::cout << std::setw(3) << kexp[ivar];
    std::cout << "]";
#endif
    for( unsigned ivar=0; ivar<_nvar; ivar++ )
      termbern *= trmat[jexp[ivar]*(_nord+1)+kexp[ivar]];
    coefbern += termbern;      
#ifdef MC__CVAR_DEBUG_BERNSTEIN
    std::cout << "  " << termbern << "  " << coefbern << std::endl;
#endif
  }
#ifdef MC__CVAR_DEBUG_BERNSTEIN2
  std::cout << std::endl;
#endif
  return coefbern;
}

template <typename T> template <typename C, typename U> inline U
CModel<T>::_polybound
( const C*coefmon, const U*const*bndbasis, const int type,
  const std::set<unsigned>&ndxmon )
{
  U* bndord = _get_bndord( coefmon, bndbasis, ndxmon );
  U bndpol;
  switch( type ){
  case Options::NAIVE:
    bndpol = _polybound_naive( coefmon, bndord, ndxmon );
    break;
  case Options::BERNSTEIN:
    bndpol = _polybound_bernstein( coefmon, bndord, bndbasis, ndxmon );
    break;
  case Options::EIGEN:
    bndpol = _polybound_eigen( coefmon, bndord, bndbasis, ndxmon );
    break;
  case Options::HYBRID:{
    U bndpol_LSB = _polybound_LSB( coefmon, bndord, bndbasis, ndxmon ),
      bndpol_eigen = _polybound_eigen( coefmon, bndord, bndbasis, ndxmon );
    if( !Op<U>::inter( bndpol, bndpol_LSB, bndpol_eigen ) )
      bndpol = bndpol_LSB;//: bndpol_eigen;
    break;
   }
  case Options::LSB: default:
    bndpol = _polybound_LSB( coefmon, bndord, bndbasis, ndxmon );
    break;
  }
  delete[] bndord;
  return bndpol;
}

////////////////////////////////// CVar ///////////////////////////////////////

template <typename T> inline CVar<T>&
CVar<T>::operator =
( const CVar<T>&CV )
{
  _CM = CV._CM;
  _set( CV );
#ifdef  MC__CMODEL_CHECK_PMODEL
  if( _CM != dynamic_cast< CModel<T>* >( PolyVar<T>::_PM ) ) assert( false );
#endif
  return *this;
}

template <typename T> inline
CVar<T>::CVar
( const double d )
: PolyVar<T>(), _CM( 0 )
{
  _coefmon[0] = d;
  *_bndrem = 0.;
  _set_bndpol( d );
}

template <typename T> inline CVar<T>&
CVar<T>::operator =
( const double d )
{
  if( _CM ){ _CM = 0; _resize( _CM ); }
  _coefmon[0] = d;
  *_bndrem = 0.;
  _set_bndpol( d );
  _unset_bndT();
  return *this;
}

template <typename T> inline
CVar<T>::CVar
( const T&B )
: PolyVar<T>(), _CM( 0 )
{
  _coefmon[0] = Op<T>::mid(B);
  *_bndrem = B-_coefmon[0];
  _set_bndpol( _coefmon[0] );
}

template <typename T> inline CVar<T>&
CVar<T>::operator =
( const T&B )
{
  if( _CM ){ _CM = 0; _resize( _CM ); }
  _coefmon[0] = Op<T>::mid(B);
  *_bndrem = B-_coefmon[0];
  _set_bndpol( _coefmon[0] );
  _unset_bndT();
  return *this;
}

template <typename T> inline
CVar<T>::CVar
( CModel<T>*CM, const double d )
: PolyVar<T>( CM ), _CM( CM )
{
  if( !_CM ) throw typename CModel<T>::Exceptions( CModel<T>::Exceptions::INIT );
  _coefmon[0] = d; for( unsigned i=1; i<nmon(); i++ )  _coefmon[i] = 0.;
  if( _CM->sparse() ) _ndxmon.insert(0);
  _bndord[0] = d; for( unsigned i=1; i<nord()+2; i++) _bndord[i]  = 0.;
  _bndord_uptd = true;
  _set_bndpol( d );
}

template <typename T> inline
CVar<T>::CVar
( CModel<T>*CM, const T&B )
: PolyVar<T>( CM ), _CM( CM )
{
  if( !_CM ) throw typename CModel<T>::Exceptions( CModel<T>::Exceptions::INIT );
  _coefmon[0] = Op<T>::mid(B); for( unsigned i=1; i<nmon(); i++ ) _coefmon[i] = 0.;
  if( _CM->sparse() ) _ndxmon.insert(0);
  _bndord[0] = _coefmon[0]; for( unsigned i=1; i<=nord(); i++) _bndord[i] = 0.;
  _bndord_uptd = true;
  *_bndrem = B-_coefmon[0];
  _set_bndpol( _coefmon[0] );
}

// ==> account for sparse format

template <typename T> template <typename U> inline
CVar<T>::CVar
( CModel<T>*&CM, const CVar<U>&CV )
: PolyVar<T>( CM ), _CM( CM )
{
  CVar<U> CVtrunc( CV );
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
CVar<T>::CVar
( CModel<T>*&CM, const CVar<U>&CV, T (*method)( const U& ) )
: PolyVar<T>( CM ), _CM( CM )
{
  if( !method ) throw typename CModel<T>::Exceptions( CModel<T>::Exceptions::INIT );
  CVar<U> CVtrunc( CV );
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
CVar<T>::CVar
( CModel<T>*&CM, const CVar<U>&CV, const T& (U::*method)() const )
: PolyVar<T>( CM ), _CM( CM )
{
  if( !method ) throw typename CModel<T>::Exceptions( CModel<T>::Exceptions::INIT );
  CVar<U> CVtrunc( CV );
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

template <typename T> inline
CVar<T>::CVar
( CModel<T>*CM, const unsigned ivar, const T&X )
: PolyVar<T>( CM ), _CM( CM )
{
  _set( ivar, X );
}

template <typename T> inline CVar<T>&
CVar<T>::_set
( const unsigned ivar, const T&X )
{
  if( !_CM ) throw typename CModel<T>::Exceptions( CModel<T>::Exceptions::INIT );

  // Scale variables and keep track of them in CModel
  double scal = Op<T>::diam(X)/2., ref = Op<T>::mid(X);
  if( isequal( scal, 0. ) ) scal = 1.;
  _CM->_set_bndpow( ivar, X, ref, scal );
  _CM->_set_bndmon();

  // Populate _coefmon w/ CVar coefficients
  _coefmon[0] = ref;
  for( unsigned i=1; i<nmon(); i++ ) _coefmon[i] = 0.;
  if( nord() > 0 ) _coefmon[nvar()-ivar] = Op<T>::diam(X)/2.;//scaling;
  if( _CM->sparse() ){ if( ref != 0. ) _ndxmon.insert(0); _ndxmon.insert(nvar()-ivar); }

  // Populate _bndord w/ bounds on CVar terms
  _bndord[0] = _coefmon[0];
  if( nord() ){
    _bndord[1] = X-_coefmon[0];
    for( unsigned i=2; i<nord()+2; i++) _bndord[i] = 0.;
    _set_bndpol( X );
  }
  else{ // 0th-order Chebyshev model
    *_bndrem = X-_coefmon[0];
    _set_bndpol( _coefmon[0] );
  }
  _bndord_uptd = true;
  _unset_bndT();
  if( _CM->options.MIXED_IA ) _set_bndT( X );

  //std::cout << *this;
  return *this;
}

template <typename T> inline void
CVar<T>::_update_bndord() const
{
  if( !_CM || _bndord_uptd ) return;
  _CM->_set_bndmon();
  _bndord[0] = (_ndxmon.empty() || _ndxmon.find(0)!=_ndxmon.end())? _coefmon[0]: 0.;
  for( unsigned i=1; i<=nord(); i++ ){
    _bndord[i] = 0.;
    for( auto it=_ndxmon.lower_bound(_posord(i)); it!=_ndxmon.lower_bound(_posord(i+1)); ++it )
      _bndord[i] += _coefmon[*it] * _bndmon(*it);
    for( unsigned j=_posord(i); _ndxmon.empty() && j<_posord(i+1); j++ )
      _bndord[i] += _coefmon[j] * _bndmon(j);
  }
  _bndord_uptd = true;
}

template <typename T> inline T
CVar<T>::_polybound
( const int type ) const
{
  if( !_CM ) return _coefmon[0];

  _update_bndord();
  T bndpol;
  switch( type ){
  case CModel<T>::Options::NAIVE:
    bndpol = _CM->_polybound_naive( _coefmon, _bndord, _ndxmon );
    break;
  case CModel<T>::Options::BERNSTEIN:
    bndpol = _CM->_polybound_bernstein( _coefmon, _bndord, _bndpow(), _ndxmon );
    break;
  case CModel<T>::Options::EIGEN:
    bndpol = _CM->_polybound_eigen( _coefmon, _bndord, _bndpow(), _ndxmon );
    break;
  case CModel<T>::Options::HYBRID:{
    T bndpol_LSB = _CM->_polybound_LSB( _coefmon, _bndord, _bndpow(), _ndxmon ),
      bndpol_eigen = _CM->_polybound_eigen( _coefmon, _bndord, _bndpow(), _ndxmon );
    if( !Op<T>::inter( bndpol, bndpol_LSB, bndpol_eigen ) )
      bndpol = Op<T>::diam(bndpol_LSB) < Op<T>::diam(bndpol_eigen)? bndpol_LSB: bndpol_eigen;
    break;
   }
  case CModel<T>::Options::LSB: default:
    bndpol = _CM->_polybound_LSB( _coefmon, _bndord, _bndpow(), _ndxmon );
    break;
  }
  return bndpol;
}

template <typename T> inline double
CVar<T>::polynomial
( const double*x ) const
{
  // ??Is there a multivariate version of Clenshaw??
  // Separate constant term in case polynomial environment is NULL
  double Pval = (_ndxmon.empty() || _ndxmon.find(0)!=_ndxmon.end())? _coefmon[0]: 0.;
  if( !_CM ) return Pval;
  // Case: sparse storage
  for( auto it=_ndxmon.begin(); it!=_ndxmon.end(); ++it ){
    if( !*it ) continue; // To avoid double-counting of the constant term
    const unsigned *iexp = _expmon(*it);
    double valmon = 1.;
    for( unsigned k=0; k<nvar(); k++ ){
      if( !iexp[k] ) continue;
      if( iexp[k] == 1 ) valmon *= (x[k]-_refvar(k))/_scalvar(k);
      else valmon *= mc::cheb((x[k]-_refvar(k))/_scalvar(k),iexp[k]);
    }
    Pval += _coefmon[*it] * valmon;
  }
  // Case: dense storage
  for( unsigned i=1; _ndxmon.empty() && i<nmon(); i++ ){
    const unsigned *iexp = _expmon(i);
    double valmon = 1.;
    for( unsigned k=0; k<nvar(); k++ ){
      if( !iexp[k] ) continue;
      if( iexp[k] == 1 ) valmon *= (x[k]-_refvar(k))/_scalvar(k);
      else valmon *= mc::cheb((x[k]-_refvar(k))/_scalvar(k),iexp[k]);
    }
    Pval += _coefmon[i] * valmon;
    //std::cout << Pval << " += " << _coefmon[i] << " x " << valmon << std::endl;
  }
  return Pval;
}

template <typename T> inline double
CVar<T>::constant( const bool reset )
{
  const double coefconst = (_ndxmon.empty() || _ndxmon.find(0)!=_ndxmon.end())? _coefmon[0]: 0.;
  if( reset ) *this -= coefconst;
  return coefconst;
}

template <typename T> inline double*
CVar<T>::linear
() const
{
  if( !nvar() || !nord() ) return 0;

  double*plin = new double[nvar()];
  for( unsigned i=0; i<nvar(); i++ )
    plin[i] = (_ndxmon.empty() || _ndxmon.find(nvar()-i)!=_ndxmon.end())?
              _coefmon[nvar()-i]/_scalvar(i): 0.;
  return plin;
}

template <typename T> inline double
CVar<T>::linear
( const unsigned ivar, const bool reset )
{
  if( ivar>=nvar() || !nord() ) return 0.;
  const double coeflin = (_ndxmon.empty() || _ndxmon.find(nvar()-ivar)!=_ndxmon.end())?
                         _coefmon[nvar()-ivar]/_scalvar(ivar): 0.;
  if( reset ){
    _coefmon[nvar()-ivar] = 0.; _bndord_uptd = false; _unset_bndpol();
    _unset_bndT();
    //if( _bndT ) *_bndT -= _coefmon[nvar()-ivar] * _bndmon(nvar()-ivar); 
  }
  return coeflin;
}

template <typename T> inline CVar<T>*
CVar<T>::_single
( const unsigned ivar )
{
  CVar<T>* CVcoef = new CVar<T>[nord()+1];
  for( unsigned q=0; q<nord()+1; q++ ) CVcoef[q].set( _CM, true );

  // Separate constant term in case polynomial environment is NULL
  CVcoef[0]._coefmon[0] = (_ndxmon.empty() || _ndxmon.find(0)!=_ndxmon.end())? _coefmon[0]: 0.;
  if( !_CM ) return CVcoef;
  unsigned*kappa = new unsigned[nvar()];
  // Case: sparse storage
  for( auto it=_ndxmon.begin(); it!=_ndxmon.end(); ++it ){
    if( !*it ) continue; // To avoid double-counting of the constant term
    for( unsigned i=0; i<nvar(); i++ ) kappa[i] = _expmon(*it)[i];
    const unsigned q = kappa[ivar];
    kappa[ivar] = 0;
    CVcoef[q]._coefmon[_loc_expmon(kappa)] = _coefmon[*it];
  }
  // Case: dense storage
  for( unsigned k=1; _ndxmon.empty() && k<nmon(); k++ ){
    for( unsigned i=0; i<nvar(); i++ ) kappa[i] = _expmon(k)[i];
    const unsigned q = kappa[ivar];
    kappa[ivar] = 0;
    CVcoef[q]._coefmon[_loc_expmon(kappa)] = _coefmon[k];
  }
  delete[] kappa;

  return CVcoef;
}

template <typename T> inline CVar<T>
CVar<T>::scale
( const T*X ) const
{
  CVar<T> CVscal( *this );
  // Return *this if null pointer to model _CM or variable ranges X
  for( unsigned i=0; _CM && X && i<nvar(); i++ ){
    // Nothing to do for variable i if X[i] = [-1,1]
    if( Op<T>::l(X[i]) == -1 && Op<T>::u(X[i]) == 1 ) continue;
    // Perform caling for variable i
    CVar<T> CVinner( _CM ); CVinner._set( i, X[i] );
    //std::cout << "CVinner[" << i << ":" << CVinner;
    CVinner = CVinner._rescale( _scalvar(i), _refvar(i) );
    //std::cout << "CVinner[" << i << "]:" << CVinner;
    CVar<T>*CVcoefi = CVscal._single( i );
    CVscal = CVinner._composition( CVcoefi );
    CVscal += *_bndrem;
    delete[] CVcoefi;
  }
  return CVscal;
}

template <typename T> inline std::ostream&
operator<<
( std::ostream&out, const CVar<T>&CV )
{
  out << std::endl
      << std::scientific << std::setprecision(5)
      << std::right;

  // Constant model
  if( !CV._CM ){
    out << "   a0    = " << std::right << std::setw(12) << CV._coefmon[0]
        << std::endl
        << "   R     = " << *(CV._bndrem) << std::endl;
  }

  // Chebyshev coefficients and corresponding exponents
  else{
    out << std::setprecision(CV._CM->options.DISPLAY_DIGITS);
    for( auto it=CV.ndxmon().begin(); it!=CV.ndxmon().end(); ++it ){
      out << "   a" << std::left << std::setw(4) << *it << " = "
          << std::right << std::setw(CV._CM->options.DISPLAY_DIGITS+7)
	  << CV._coefmon[*it] << "   ";
      for( unsigned k=0; k<CV.nvar(); k++ )
        out << std::setw(3) << CV._expmon(*it)[k];
      out << std::endl;
    }
    for( unsigned i=0; CV.ndxmon().empty() && i<CV.nmon(); i++ ){
      out << "   a" << std::left << std::setw(4) << i << " = "
          << std::right << std::setw(CV._CM->options.DISPLAY_DIGITS+7)
	  << CV._coefmon[i] << "   ";
      for( unsigned k=0; k<CV.nvar(); k++ )
        out << std::setw(3) << CV._expmon(i)[k];
      out << std::endl;
    }
    // Remainder term
    out << std::right << "   R     =  " << *(CV._bndrem)
        << std::endl;
  }

  // Range bounder
  out << std::right << "   B     =  " << CV.B()
      << std::endl;

  return out;
}

template <typename T> inline CVar<T>
operator+
( const CVar<T>&CV )
{
  return CV;
}

template <typename T> template <typename U> inline CVar<T>&
CVar<T>::operator+=
( const CVar<U>&CV )
{
  if( !CV._CM ){
    if( _ndxmon.empty() || !_ndxmon.insert(0).second )
      _coefmon[0] += CV._coefmon[0]; // update
    else
      _coefmon[0] =  CV._coefmon[0]; // append
    if( _CM && _bndord_uptd ) _bndord[0] += CV._coefmon[0];
    *_bndrem += *(CV._bndrem);
    if( _bndpol ) *_bndpol += CV._coefmon[0];
    if( _bndT )   *_bndT   += CV._coefmon[0] + *CV._bndrem;
  }
  else if( !_CM ){
    CVar<T> CV2(*this);
    *this = CV; *this += CV2;
  }
  else{
    if( _CM != CV._CM )
      throw typename CModel<T>::Exceptions( CModel<T>::Exceptions::CMODEL );
    // Case: sparse storage
    for( auto it=CV.ndxmon().begin(); it!=CV.ndxmon().end(); ++it ){
      if( _ndxmon.empty() || !_ndxmon.insert(*it).second )
        _coefmon[*it] += CV._coefmon[*it]; // update
      else
        _coefmon[*it] =  CV._coefmon[*it]; // append
    }
    // Case: dense storage
    for( unsigned i=0; CV._ndxmon.empty() && i<nmon(); i++ )
      if( _ndxmon.empty() || _ndxmon.find(i)!=_ndxmon.end() )
        _coefmon[i] += CV._coefmon[i]; // update
      else
        _coefmon[i] =  CV._coefmon[i]; // append
    // Switch to dense mode when filled
    if( CV._ndxmon.empty() || _ndxmon.size()==nmon() ) _ndxmon.clear(); 
    _bndord_uptd = false;
    *_bndrem += *(CV._bndrem);
    _unset_bndpol();
    if( _bndT && CV._bndT ) *_bndT += *CV._bndT;
    else _unset_bndT();
  }
  return *this;
}

template <typename T, typename U> inline CVar<T>
operator+
( const CVar<T>&CV1, const CVar<U>&CV2 )
{
  CVar<T> CV3( CV1 );
  CV3 += CV2;
  return CV3;
}

template <typename T> inline CVar<T>&
CVar<T>::operator +=
( const double c )
{
  if( _ndxmon.empty() || !_ndxmon.insert(0).second )
    _coefmon[0] += c; // update
  else
    _coefmon[0] =  c; // append
  if( _CM && _bndord_uptd ) _bndord[0] += c;
  if( _bndpol ) *_bndpol += c;
  if( _bndT )   *_bndT += c;
  return *this;
}

template <typename T> inline CVar<T>
operator+
( const CVar<T>&CV1, const double c )
{
  CVar<T> CV3( CV1 );
  CV3 += c;
  return CV3;
}

template <typename T> inline CVar<T>
operator+
( const double c, const CVar<T>&CV2 )
{
  CVar<T> CV3( CV2 );
  CV3 += c;
  return CV3;
}

template <typename T> template <typename U> inline CVar<T>&
CVar<T>::operator+=
( const U&I )
{
  *_bndrem += I;
  _center();
  if( _bndT ) *_bndT += I;
  return *this;
}

template <typename T> inline CVar<T>
operator+
( const CVar<T>&CV1, const T&I )
{
  CVar<T> CV3( CV1 );
  CV3 += I;
  return CV3;
}

template <typename T> inline CVar<T>
operator+
( const T&I, const CVar<T>&CV2 )
{
  CVar<T> CV3( CV2 );
  CV3 += I;
  return CV3;
}

template <typename T> inline CVar<T>
operator-
( const CVar<T>&CV )
{
  if( !CV._CM ){
    CVar<T> CV2;
    CV2._coefmon[0] = -CV._coefmon[0];
    *CV2._bndrem = - *CV._bndrem;
    if( CV._bndpol ) CV2._set_bndpol( - *CV._bndpol );
    if( CV._bndT )   CV2._set_bndT( - *CV._bndT );
    return CV2;
  }
  CVar<T>& CV2 = *CV._CV();  // Use internal CM variable to save time
  CV2._unset_bndpol(); CV2._unset_bndT(); CV2._bndord_uptd = false;
  CV2._ndxmon = CV._ndxmon;
  // Case: sparse storage
  for( auto it=CV._ndxmon.begin(); it!=CV._ndxmon.end(); ++it )
    CV2._coefmon[*it] = -CV._coefmon[*it];
  // Case: dense storage
  for( unsigned i=0; CV._ndxmon.empty() && i<CV.nmon(); i++ )
    CV2._coefmon[i] = -CV._coefmon[i];
  *CV2._bndrem = - *CV._bndrem;
  if( CV._bndord_uptd ){
    for( unsigned i=0; i<=CV.nord(); i++ ) CV2._bndord[i] = -CV._bndord[i];
    CV2._bndord_uptd = true;
  }
  if( CV._bndpol ) CV2._set_bndpol( - *CV._bndpol );
  if( CV._bndT )   CV2._set_bndT( - *CV._bndT );
  return CV2;
}

template <typename T> template <typename U> inline CVar<T>&
CVar<T>::operator-=
( const CVar<U>&CV )
{
  if( !CV._CM ){
    if( _ndxmon.empty() || !_ndxmon.insert(0).second )
      _coefmon[0] -= CV._coefmon[0]; // update
    else
      _coefmon[0] = -CV._coefmon[0]; // append
    if( _CM && _bndord_uptd ) _bndord[0] -= CV._coefmon[0];
    *_bndrem -= *(CV._bndrem);
    if( _bndpol ) *_bndpol -= CV._coefmon[0];
    if( _bndT )   *_bndT   -= CV._coefmon[0] + *CV._bndrem;
  }
  else if( !_CM ){
    CVar<T> CV2(*this);
    *this = -CV; *this += CV2;
  }
  else{
    if( _CM != CV._CM )
      throw typename CModel<T>::Exceptions( CModel<T>::Exceptions::CMODEL );
    // Case: sparse storage
    for( auto it=CV.ndxmon().begin(); it!=CV.ndxmon().end(); ++it ){
      if( _ndxmon.empty() || !_ndxmon.insert(*it).second )
        _coefmon[*it] -= CV._coefmon[*it]; // update
      else
        _coefmon[*it] = -CV._coefmon[*it]; // append
    }
    // Case: dense storage
    for( unsigned i=0; CV._ndxmon.empty() && i<nmon(); i++ )
      if( _ndxmon.empty() || _ndxmon.find(i)!=_ndxmon.end() )
        _coefmon[i] -= CV._coefmon[i]; // update
      else
        _coefmon[i] = -CV._coefmon[i]; // append
    // Switch to dense mode when filled
    if( CV._ndxmon.empty() || _ndxmon.size()==nmon() ) _ndxmon.clear(); 
    _bndord_uptd = false;
    *_bndrem -= *(CV._bndrem);
    _unset_bndpol();
    if( _bndT && CV._bndT ) *_bndT -= *CV._bndT;
    else _unset_bndT();

  }
  return *this;
}

template <typename T, typename U> inline CVar<T>
operator-
( const CVar<T>&CV1, const CVar<U>&CV2 )
{
  CVar<T> CV3( CV1 );
  CV3 -= CV2;
  return CV3;
}

template <typename T> inline CVar<T>&
CVar<T>::operator-=
( const double c )
{
  if( _ndxmon.empty() || !_ndxmon.insert(0).second )
    _coefmon[0] -= c; // update
  else
    _coefmon[0] = -c; // append
  if( _CM && _bndord_uptd ) _bndord[0] -= c;
  if( _bndpol ) *_bndpol -= c;
  if( _bndT )   *_bndT -= c;
  return *this;
}

template <typename T> inline CVar<T>
operator-
( const CVar<T>&CV1, const double c )
{
  CVar<T> CV3( CV1 );
  CV3 -= c;
  return CV3;
}

template <typename T> inline CVar<T>
operator-
( const double c, const CVar<T>&CV2 )
{
  CVar<T> CV3( -CV2 );
  CV3 += c;
  return CV3;
}

template <typename T> template <typename U> inline CVar<T>&
CVar<T>::operator-=
( const U&I )
{
  *_bndrem -= I;
  _center();
  if( _bndT ) *_bndT -= I;
  return *this;
}

template <typename T> inline CVar<T>
operator-
( const CVar<T>&CV1, const T&I )
{
  CVar<T> CV3( CV1 );
  CV3 -= I;
  return CV3;
}

template <typename T> inline CVar<T>
operator-
( const T&I, const CVar<T>&CV2 )
{
  CVar<T> CV3( -CV2 );
  CV3 += I;
  return CV3;
}

template <typename T> inline CVar<T>&
CVar<T>::operator*=
( const CVar<T>&CV )
{
   CVar<T> CV2( *this );
   *this = CV * CV2;
   return *this;
}

template <typename T> inline CVar<T>
operator*
( const CVar<T>&CV1, const CVar<T>&CV2 )
{
  if( !CV2._CM ) return( CV1 * CV2._coefmon[0] + CV1 * *(CV2._bndrem) );
  if( !CV1._CM ) return( CV2 * CV1._coefmon[0] + CV2 * *(CV1._bndrem) );
  if( &CV1 == &CV2 ) return sqr(CV1);
  if( !CV1._ndxmon.empty() && CV2._ndxmon.empty() ) return( CV2 * CV1 );

  if( CV1._CM != CV2._CM )
    throw typename CModel<T>::Exceptions( CModel<T>::Exceptions::CMODEL );
  CVar<T>& CV3 = *CV1._CV();
  for( unsigned i=0; i<CV3.nmon(); i++ ) CV3._coefmon[i] = 0.;
  CV3._ndxmon.clear();

  // Populate _coefmon and _bndrem for product of polynomial parts
  double coefrem = 0.;
#ifdef MC__CVAR_SPARSE_PRODUCT_NAIVE
  // Case: sparse storage
  for( auto it=CV1._ndxmon.begin(); it!=CV1._ndxmon.end(); ++it ){
    for( auto jt=CV2._ndxmon.begin(); jt!=CV2._ndxmon.end(); ++jt ){
      const unsigned*prodij = *it<*jt? CV3._prodmon(*jt,*it): CV3._prodmon(*it,*jt);
      for( unsigned k=0; k<prodij[1]; k++ ){
        CV3._coefmon[prodij[2+k]] += CV1._coefmon[*it] * CV2._coefmon[*jt]
          / (double)prodij[0];
        CV3._ndxmon.insert( prodij[2+k] );
      }
      coefrem += std::fabs( CV1._coefmon[*it] * CV2._coefmon[*jt] )
          * ( 1. - (double)prodij[1] / (double)prodij[0] );
    }
  }
#else
  CV1._sprod( CV1, CV2, CV3._coefmon, CV3._ndxmon, coefrem );
#endif
  // Case: dense storage
  for( unsigned i=0; CV1._ndxmon.empty() && i<CV1.nmon(); i++ ){
    // dense x sparse
    for( auto jt=CV2._ndxmon.begin(); jt!=CV2._ndxmon.end(); ++jt ){
      const unsigned int*prodij = i<*jt? CV3._prodmon(*jt,i): CV3._prodmon(i,*jt);
      for( unsigned k=0; k<prodij[1]; k++ )
        CV3._coefmon[prodij[2+k]] += CV1._coefmon[i] * CV2._coefmon[*jt]
          / (double)prodij[0];
      coefrem += std::fabs( CV1._coefmon[i] * CV2._coefmon[*jt] )
          * ( 1. - (double)prodij[1] / (double)prodij[0] );
    }
    if( !CV2._ndxmon.empty() ) continue;
    // dense x dense
    for( unsigned j=0; j<i; j++ ){
      const unsigned int*prodij = CV3._prodmon(i,j);
      for( unsigned k=0; k<prodij[1]; k++ )
        CV3._coefmon[prodij[2+k]] += ( CV1._coefmon[i] * CV2._coefmon[j]
          + CV1._coefmon[j] * CV2._coefmon[i] ) / (double)prodij[0];
      coefrem += std::fabs( CV1._coefmon[i] * CV2._coefmon[j]
          + CV1._coefmon[j] * CV2._coefmon[i] )
          * ( 1. - (double)prodij[1] / (double)prodij[0] );
    }
    const unsigned int*prodii = CV3._prodmon(i,i);
    for( unsigned k=0; k<prodii[1]; k++ )
      CV3._coefmon[prodii[2+k]] += CV1._coefmon[i] * CV2._coefmon[i]
        / (double)prodii[0];
    coefrem += std::fabs( CV1._coefmon[i] * CV2._coefmon[i] )
        * ( 1. - (double)prodii[1] / (double)prodii[0] );
  }
  *(CV3._bndrem) = coefrem * (Op<T>::zeroone()*2.-1.);
  // Switch to dense mode when filled
  if( CV3._ndxmon.size()==CV3.nmon() ) CV3._ndxmon.clear(); 

  // Populate _coefmon and _bndrem for product of polynomial and remainder parts
  T P1 = CV1._polybound();
  T P2 = CV2._polybound();
  //T P1 = 0., P2 = 0.;
  //for( unsigned i=0; i<=CV3.nord(); i++ ){
  //  P1 += CV1._bndord[i];
  //  P2 += CV2._bndord[i];
  //}
  //T R1 = *(CV3._bndrem) + ( P1 + *(CV1._bndrem) ) * *(CV2._bndrem) + P2 * *(CV1._bndrem);
  //T R2 = *(CV3._bndrem) + P1 * *(CV2._bndrem) + ( P2 + *(CV2._bndrem) ) * *(CV1._bndrem);
  T R1 = *(CV3._bndrem) + CV1.B() * *CV2._bndrem + P2 * *CV1._bndrem;
  T R2 = *(CV3._bndrem) + P1 * *CV2._bndrem + CV2.B() * *CV1._bndrem;
  if( !Op<T>::inter( *(CV3._bndrem), R1, R2) )
    *(CV3._bndrem) = ( Op<T>::diam(R1)<Op<T>::diam(R2)? R1: R2 );

  // Update _bndord for product term (except remainder term)
  CV3._unset_bndpol();
  CV3._bndord_uptd = false;
  if( CV3._CM->options.MIXED_IA ) CV3._set_bndT( CV1.B()*CV2.B() );
  else CV3._unset_bndT();
  
  // std::cout << CV3;
  return CV3;
}

template <typename T> inline CVar<T>
sqr
( const CVar<T>&CV )
{
  if( !CV._CM ){
    CVar<T> CV2( CV );
    CV2._coefmon[0] *= CV2._coefmon[0];
    *(CV2._bndrem) *= 2. + *(CV2._bndrem);
    CV2._unset_bndpol();
    return CV2;
  }

  CVar<T>& CV2 = *CV._CV();
  for( unsigned i=0; i<CV2.nmon(); i++ ) CV2._coefmon[i] = 0.;
  CV2._ndxmon.clear();

  // Populate _coefmon and _bndrem for product of polynomial parts
  double coefrem = 0.;
#ifdef MC__CVAR_SPARSE_PRODUCT_NAIVE
  // Case: sparse storage
  for( auto it=CV._ndxmon.begin(); it!=CV._ndxmon.end(); ++it ){
    for( auto jt=CV._ndxmon.begin(); jt!=it; ++jt ){
      const unsigned*prodij = CV2._prodmon(*it,*jt);
      for( unsigned k=0; k<prodij[1]; k++ ){
        CV2._coefmon[prodij[2+k]] += 2. * CV._coefmon[*it] * CV._coefmon[*jt]
          / (double)prodij[0];
        CV2._ndxmon.insert( prodij[2+k] );
      }
      coefrem += 2. * std::fabs( CV._coefmon[*it] * CV._coefmon[*jt] )
          * ( 1. - (double)prodij[1] / (double)prodij[0] );
    }
    const unsigned*prodii = CV2._prodmon(*it,*it);
    for( unsigned k=0; k<prodii[1]; k++ ){
      CV2._coefmon[prodii[2+k]] += CV._coefmon[*it] * CV._coefmon[*it]
        / (double)prodii[0];
      CV2._ndxmon.insert( prodii[2+k] );
    }
    coefrem += std::fabs( CV._coefmon[*it] * CV._coefmon[*it] )
        * ( 1. - (double)prodii[1] / (double)prodii[0] );
  }
#else
  CV._ssqr( CV, CV2._coefmon, CV2._ndxmon, coefrem );
#endif
  // Case: dense storage
  for( unsigned i=0; CV._ndxmon.empty() && i<CV.nmon(); i++ ){
    for( unsigned j=0; j<i; j++ ){
      const unsigned*prodij = CV2._prodmon(i,j);
      for( unsigned k=0; k<prodij[1]; k++ )
        CV2._coefmon[prodij[2+k]] += 2. * CV._coefmon[i] * CV._coefmon[j]
          / (double)prodij[0];
      coefrem += 2. * std::fabs( CV._coefmon[i] * CV._coefmon[j] )
          * ( 1. - (double)prodij[1] / (double)prodij[0] );
    }
    const unsigned*prodii = CV2._prodmon(i,i);
    for( unsigned k=0; k<prodii[1]; k++ )
      CV2._coefmon[prodii[2+k]] += CV._coefmon[i] * CV._coefmon[i]
        / (double)prodii[0];
    coefrem += std::fabs( CV._coefmon[i] * CV._coefmon[i] )
        * ( 1. - (double)prodii[1] / (double)prodii[0] );
  }
  *(CV2._bndrem) = coefrem * (Op<T>::zeroone()*2.-1.);
  // Switch to dense mode when filled
  if( CV2._ndxmon.size()==CV2.nmon() ) CV2._ndxmon.clear(); 

  // Populate _coefmon and _bndrem for product of polynomial and remainder parts
  //T PB = 0.;
  //for( unsigned i=0; i<=CV3.nord(); i++ ) PB += CV._bndord[i];
  T PB = CV._polybound();
  //*(CV2._bndrem) += ( 2. * PB + *(CV._bndrem) ) * *(CV._bndrem);
  *(CV2._bndrem) += ( PB + CV.B() ) * *(CV._bndrem);

  // Populate _bndord for product term (except remainder term)
  CV2._unset_bndpol();
  CV2._bndord_uptd = false;
  if( CV2._CM->options.MIXED_IA ) CV2._set_bndT( Op<T>::sqr(CV.B()) );
  else CV2._unset_bndT();
  return CV2;
}

template <typename T> inline CVar<T>&
CVar<T>::operator*=
( const double c )
{
  if( !_CM ){
    _coefmon[0] *= c;
    *(_bndrem) *= c;
    if( _bndpol ) *_bndpol *= c;
    return *this;
  }
  // Case: Sparse storage
  for( auto it=_ndxmon.begin(); it!=_ndxmon.end(); ++it )
    _coefmon[*it] *= c;
  // Case: dense storage
  for( unsigned i=0; _ndxmon.empty() && i<nmon(); i++ )
    _coefmon[i] *= c;
  for( unsigned i=0; _bndord_uptd && i<=nord(); i++ )
    _bndord[i] *= c;
  *_bndrem *= c;
  if( _bndpol ) *_bndpol *= c;
  if( _bndT ) *_bndT *= c;
  return *this;
}

template <typename T> inline CVar<T>
operator*
( const CVar<T>&CV1, const double c )
{
  CVar<T> CV3( CV1 );
  CV3 *= c;
  return CV3;
}

template <typename T> inline CVar<T>
operator*
( const double c, const CVar<T>&CV2 )
{
  CVar<T> CV3( CV2 );
  CV3 *= c;
  return CV3;
}

template <typename T> inline CVar<T>&
CVar<T>::operator*=
( const T&I )
{
  if( !_CM ){
    *(_bndrem) += _coefmon[0];
    _coefmon[0] = 0.;
    *(_bndrem) *= I;
  }
  else{
    const double Imid = Op<T>::mid(I);
    T Icur = bound();
    // Case: Sparse storage
    for( auto it=_ndxmon.begin(); it!=_ndxmon.end(); ++it )
      _coefmon[*it] *= Imid;
    // Case: dense storage
    for( unsigned i=0; _ndxmon.empty() && i<nmon(); i++ )
      _coefmon[i] *= Imid;
    for( unsigned i=0; _bndord_uptd && i<=nord(); i++ )
      _bndord[i] *= Imid;
    *_bndrem *= Imid;
    *_bndrem += (I-Imid)*Icur;
  }
  _unset_bndpol();
  if( _bndT ) *_bndT *= I;
  return (*this);
}

template <typename T> inline CVar<T>
operator*
( const CVar<T>&CV1, const T&I )
{
  CVar<T> CV3( CV1 );
  CV3 *= I;
  return CV3;
}

template <typename T> inline CVar<T>
operator*
( const T&I, const CVar<T>&CV2 )
{
  CVar<T> CV3( CV2 );
  CV3 *= I;
  return CV3;
}

template <typename T> inline CVar<T>&
CVar<T>::operator /=
( const CVar<T>&CV )
{
   *this *= inv(CV);
   return *this;
}

template <typename T> inline CVar<T>
operator /
( const CVar<T>&CV1, const CVar<T>&CV2 )
{
  return CV1 * inv(CV2);
}

template <typename T> inline CVar<T>&
CVar<T>::operator /=
( const double c )
{
  if( isequal( c, 0. ) )
    throw typename CModel<T>::Exceptions( CModel<T>::Exceptions::DIV );
  *this *= (1./c);
  return *this;
}

template <typename T> inline CVar<T>
operator /
( const CVar<T>&CV, const double c )
{
  if ( isequal( c, 0. ))
    throw typename CModel<T>::Exceptions( CModel<T>::Exceptions::DIV );
  return CV * (1./c);
}

template <typename T> inline CVar<T>
operator /
( const double c, const CVar<T>&CV )
{
  return inv(CV) * c;
}
/*
template <typename T> inline CVar<T>
inv
( const CVar<T>&CV )
{
  if( !CV._CM )
    return CVar<T>( Op<T>::inv(CV._coefmon[0] + *(CV._bndrem)) );

  double b(Op<T>::mid(CV.B())), a(Op<T>::u(CV.B())-b), z0sq(-b/a), z0(std::sqrt(z0sq*z0sq-1.)),
         az0(b*b-a*a), invz0sq(-a/b), an, rem;
  CVar<T> CV2( CV._CM, 0. ), CV3( CV._CM, 0. ), CVm2 = CV._rescale(a,b);

  if(z0sq < 0) z0sq -= z0;
  else {z0sq += z0; z0*=-1.;}

  an = 2./az0*std::pow(invz0sq,-double(CV.nord()));
  //std::cout << "an: " << an << std::endl;

  //remainder based on sum of geometric series
  rem = an/(std::abs(z0sq)-1.);

  //composition based on http://en.wikipedia.org/wiki/Clenshaw_algorithm#Special_case_for_Chebyshev_series
  if (CV.nord() == 0)
    CV2 = an/2.;
  else if (CV.nord() == 1)
    CV2 = an*z0sq/2. + CVm2*an;
  else{
    CV2 = an;
    an *= z0sq;
    CV3 = an + CVm2*CV2;

    if (CV.nord() > 2) for( unsigned i=CV.nord()-2; i>1; i-=2 ){
      an *= z0sq;
      CV2 = an + CVm2*CV3 - CV2;
      an *= z0sq;
      CV3 = an + CVm2*CV2 - CV3;
      }

    if (CV.nord()%2 == 0) {
      an *= 0.5*z0sq;
      CV2 = an + (CVm2/2.)*CV3 - CV2;
    }
    else {
      an *= z0sq;
      CV2 = an + CVm2*CV3 - CV2;
      an *= 0.5*z0sq;
      CV2 = an + (CVm2/2.)*CV2 - CV3;
    }
  }

  CV2 += T(-rem,rem);

  //CV2._bndord_uptd = false;
  //CV2._unset_bndpol();
  //if( CV2._CM->options.CENTER_REMAINDER ) CV2._center(); //crashes on order 0 for some reason.
  return CV2;
}
*/

template <typename T> inline CVar<T>
inv
( const CVar<T>&CV )
{
  if( !CV._CM )
    return CVar<T>( Op<T>::inv(CV._coefmon[0] + *(CV._bndrem)) );
  if ( Op<T>::l(CV.B()) <= 0. && Op<T>::u(CV.B()) >= 0. )
    throw typename CModel<T>::Exceptions( CModel<T>::Exceptions::INV );

  CVar<T> CVI( CV._CM, 0. ), CV2( CV._CM, 0. );
  //double coefmon[CV.nord()+1];
  double* coefmon = CV._coefinterp();
  CV._interpolation( coefmon, mc::inv );

  double m(Op<T>::mid(CV.B())), r(Op<T>::u(CV.B())-m), rem;
#ifndef MC__CVAR_FORCE_REM_DERIV
  double ub(0), lb(0);
  for (unsigned i(0); i<=CV.nord(); i++) {
    ub += coefmon[i];
    lb += std::pow(-1.,i)*coefmon[i];
  }
  rem = std::max(std::fabs(mc::inv(r+m)-ub), std::fabs(mc::inv(m-r)-lb));
#else
  rem = 4.*std::pow(r,double(CV.nord()+2));
#endif

  CVI = CV._rescale(r,m);
  CV2 = CVI._composition( coefmon );
  //CV2 += T(-rem, rem);
  CV2 += (2.*Op<T>::zeroone()-1.)*rem;

  if( CV._CM->options.MIXED_IA ) CV2._set_bndT( Op<T>::inv(CV.B()) );
  return CV2;
}

template <typename T> inline CVar<T>
sqrt
( const CVar<T>&CV )
{
  if( !CV._CM )
    return CVar<T>( Op<T>::sqrt(CV._coefmon[0] + *(CV._bndrem)) );
  if ( Op<T>::l(CV.B()) < 0. )
    throw typename CModel<T>::Exceptions( CModel<T>::Exceptions::SQRT );

  CVar<T> CVI( CV._CM, 0. ), CV2( CV._CM, 0. );
  //double coefmon[CV.nord()+1];
  double* coefmon = CV._coefinterp();
  CV._interpolation( coefmon, std::sqrt );

  double b(Op<T>::mid(CV.B())), a(Op<T>::u(CV.B())-b), rem, ub(0), lb(0);
  for (unsigned i(0); i<=CV.nord(); i++) {
    ub += coefmon[i];
    lb += i%2? -coefmon[i]: coefmon[i];
  }
  rem = std::max(std::fabs(std::sqrt(a+b)-ub), std::fabs(std::sqrt(b-a)-lb));

  CVI = CV._rescale(a,b);
  CV2 = CVI._composition( coefmon );
  //CV2 += T(-rem, rem);
  CV2 += (2.*Op<T>::zeroone()-1.)*rem;

  if( CV._CM->options.MIXED_IA ) CV2._set_bndT( Op<T>::sqrt(CV.B()) );
  return CV2;
}

template <typename T> inline CVar<T>
exp
( const CVar<T>&CV )
{ 
  if( !CV._CM )
    return CVar<T>( Op<T>::exp(CV._coefmon[0] + *(CV._bndrem)) );

  CVar<T> CVI( CV._CM, 0. ), CV2( CV._CM, 0. );
  //double coefmon[CV.nord()+1];
  double* coefmon = CV._coefinterp();
  CV._interpolation( coefmon, std::exp );

  double m(Op<T>::mid(CV.B())), r(Op<T>::u(CV.B())-m), rem;
#ifndef MC__CVAR_FORCE_REM_DERIV
  double ub(0), lb(0);
  for (unsigned i(0); i<=CV.nord(); i++) {
    ub += coefmon[i];
    lb += std::pow(-1.,i)*coefmon[i];
  }
  rem = std::max(std::fabs(std::exp(r+m)-ub), std::fabs(std::exp(m-r)-lb));
#else
  double fact(1);
  for (unsigned i(1); i<=CV.nord()+1; i++) fact *= double(i);
  double M = Op<T>::abs(Op<T>::exp(CV.B()));
  rem = 2.*M*std::pow(r/2.,double(CV.nord()+1))/fact;
#endif

  CVI = CV._rescale(r,m);
  CV2 = CVI._composition( coefmon );
  CV2 += (2.*Op<T>::zeroone()-1.)*rem;

  if( CV._CM->options.MIXED_IA ) CV2._set_bndT( Op<T>::exp(CV.B()) );
  return CV2;
}

template <typename T> inline CVar<T>
log
( const CVar<T>&CV )
{
  if( !CV._CM )
    return CVar<T>( Op<T>::log(CV._coefmon[0] + *(CV._bndrem)) );
  if ( Op<T>::l(CV.B()) <= 0. )
    throw typename CModel<T>::Exceptions( CModel<T>::Exceptions::LOG );

  CVar<T> CVI( CV._CM, 0. ), CV2( CV._CM, 0. );
  //double coefmon[CV.nord()+1];
  double* coefmon = CV._coefinterp();
  CV._interpolation( coefmon, std::log );

  double b(Op<T>::mid(CV.B())), a(Op<T>::u(CV.B())-b), rem, ub(0), lb(0);
  for (unsigned i(0); i<=CV.nord(); i++) {
    ub += coefmon[i];
    lb += std::pow(-1.,i)*coefmon[i];
  }
  rem = std::max(std::fabs(std::log(a+b)-ub), std::fabs(std::log(b-a)-lb));

  CVI = CV._rescale(a,b);
  CV2 = CVI._composition( coefmon );
  //CV2 += T(-rem, rem);
  CV2 += (2.*Op<T>::zeroone()-1.)*rem;

  if( CV._CM->options.MIXED_IA ) CV2._set_bndT( Op<T>::log(CV.B()) );
  return CV2;
}

template <typename T> inline CVar<T>
xlog
( const CVar<T>&CV )
{
  return CV * log( CV );
}

template <typename T> inline CVar<T>
pow
( const CVar<T>&CV, const int n )
{
  if( !CV._CM )
    return CVar<T>( Op<T>::pow(CV._coefmon[0] + *(CV._bndrem), n) );

  if( n < 0 ) return pow( inv( CV ), -n );
  CVar<T> CV2( CV._CM->_intpow( CV, n ) );
  if( CV._CM->options.MIXED_IA ) CV2._set_bndT( Op<T>::pow(CV.B(),n) );
  return CV2;
}

template <typename T> inline CVar<T>
pow
( const CVar<T> &CV, const double a )
{
  return exp( a * log( CV ) );
}

template <typename T> inline CVar<T>
pow
( const CVar<T> &CV1, const CVar<T> &CV2 )
{
  return exp( CV2 * log( CV1 ) );
}

template <typename T> inline CVar<T>
pow
( const double a, const CVar<T> &CV )
{
  return exp( CV * std::log( a ) );
}

template <typename T> inline CVar<T>
monomial
(const unsigned n, const CVar<T>*CV, const int*k)
{
  if( n == 0 ){
    return 1.;
  }
  if( n == 1 ){
    return pow( CV[0], k[0] );
  }
  return pow( CV[0], k[0] ) * monomial( n-1, CV+1, k+1 );
}

template <typename T> inline CVar<T>
cheb
( const CVar<T> &CV, const unsigned n )
{
  //if( Op<T>::l(CV.B()) < -1. || Op<T>::u(CV.B()) > 1. )
  //  throw typename CModel<T>::Exceptions( CModel<T>::Exceptions::CHEB );
  switch( n ){
    case 0:  return 1.;
    case 1:  return CV;
    default: break;
  }
  return 2.*(CV*cheb(CV,n-1))-cheb(CV,n-2);
  //CVar<T> CVcheb = 2.*(CV*cheb(CV,n-1))-cheb(CV,n-2);
  //return( inter( CVcheb, CVcheb, CVar<T>(I(-1.,1.)) )? CVcheb: CVar<T>(I(-1.,1.)) );
}

template <typename T> inline CVar<T>
cos
( const CVar<T> &CV )
{
  if( !CV._CM )
    return CVar<T>( Op<T>::cos(CV._coefmon[0] + *(CV._bndrem)) );

  CVar<T> CVI( CV._CM, 0. ), CV2( CV._CM, 0. );
  double* coefmon = CV._coefinterp();
  CV._interpolation( coefmon, std::cos );

  double m(Op<T>::mid(CV.B())), r(Op<T>::u(CV.B())-m), rem, fact(1);
  for (unsigned i(1); i<=CV.nord()+1; i++) fact *= double(i);
  double M = CV.nord()%2? Op<T>::abs(Op<T>::cos(CV.B())):
                          Op<T>::abs(Op<T>::sin(CV.B()));
  rem = 2.*M*std::pow(r/2.,double(CV.nord()+1))/fact;

  CVI = CV._rescale(r,m);
  CV2 = CVI._composition( coefmon );
  CV2 += (2.*Op<T>::zeroone()-1.)*rem;

  if( CV._CM->options.MIXED_IA ) CV2._set_bndT( Op<T>::cos(CV.B()) );
  return CV2;
}

template <typename T> inline CVar<T>
sin
( const CVar<T> &CV )
{
  return cos( CV - PI/2. );
}

template <typename T> inline CVar<T>
acos
( const CVar<T> &CV )
{
  if( !CV._CM )
    return CVar<T>( Op<T>::acos(CV._coefmon[0] + *(CV._bndrem)) );
  if ( Op<T>::l(CV.B()) < -1. && Op<T>::u(CV.B()) > 1. )
    throw typename CModel<T>::Exceptions( CModel<T>::Exceptions::ACOS );

  CVar<T> CVI( CV._CM, 0. ), CV2( CV._CM, 0. );
  // INCORRECT AS IMPLEMENTED -- NEEDS FIXING
  throw typename CModel<T>::Exceptions( CModel<T>::Exceptions::UNDEF );

  double coefmon[CV.nord()+1];
  double b(Op<T>::mid(CV.B())), a(Op<T>::u(CV.B())-b), rem, ub(-4.*a/PI);
  coefmon[0] = 0.5*PI*a+b;
  coefmon[1] = ub;
  for (unsigned i(3); i<=CV.nord(); i+=2) {
    coefmon[i-1] = 0.;
    coefmon[i] = coefmon[1]/std::pow(double(i),2.);
    ub += coefmon[i];
  }
  if (CV.nord()%2==0) coefmon[CV.nord()] = 0.;
  rem = a*PI/6. + ub;

  CVI = CV._rescale(a,b);
  CV2 = CVI._composition( coefmon );
  CV2 += (2.*Op<T>::zeroone()-1.)*rem;

  if( CV._CM->options.MIXED_IA ) CV2._set_bndT( Op<T>::acos(CV.B()) );
  return CV2;
}

template <typename T> inline CVar<T>
asin
( const CVar<T> &CV )
{
  return PI/2. - acos( CV );
}

template <typename T> inline CVar<T>
tan
( const CVar<T> &CV )
{
  return sin( CV ) / cos( CV );
}

template <typename T> inline CVar<T>
atan
( const CVar<T> &CV )
{
  return asin( CV / sqrt( sqr( CV ) + 1. ) );
}

template <typename T> inline CVar<T>
fabs
( const CVar<T> &CV )
{
  if( !CV._CM )
    return CVar<T>( Op<T>::fabs(CV._coefmon[0] + *(CV._bndrem)) );
  if ( Op<T>::l(CV.B()) >= 0. )
    return CV;
  if ( Op<T>::u(CV.B()) <= 0. )
    return -CV;

#ifdef MC__CVAR_FABS_SQRT
  CVar<T> CV2( sqrt( sqr(CV) ) );
  if( CV._CM->options.MIXED_IA ) CV2._set_bndT( Op<T>::fabs(CV.B()) );
  return CV2;

#else
  if( CV.nord() < 2 ){
    CVar<T> CV2( sqrt( sqr(CV) ) );
    if( CV._CM->options.MIXED_IA ) CV2._set_bndT( Op<T>::fabs(CV.B()) );
    return CV2;
  }

  CVar<T> CVI( CV._CM, 0. ), CV2( CV._CM, 0. );
  double* coefmon = CV._coefinterp();
  CV._interpolation( coefmon, std::fabs );

  double m(Op<T>::mid(CV.B())), r(Op<T>::u(CV.B())-m), rem, fact(1);
  for (unsigned i(1); i<=CV.nord()+1; i++) fact *= double(i);
  rem = 2./mc::PI*r/double(CV.nord()-1);
  //rem = 4.*std::pow(a/2.,double(CV.nord()+1))/fact;

  CVI = CV._rescale(r,m);
  CV2 = CVI._composition( coefmon );
  CV2 += (2.*Op<T>::zeroone()-1.)*rem;

  if( CV._CM->options.MIXED_IA ) CV2._set_bndT( Op<T>::fabs(CV.B()) );
  return CV2;
#endif
/*
  CVar<T> CVI( CV._CM, 0. ), CV2( CV._CM, 0. );
  //REMAINDER TO BE IMPLEMENTED
  throw typename CModel<T>::Exceptions( CModel<T>::Exceptions::UNDEF );
  //double coefmon[CV.nord()+1];
  double* coefmon = CV._coefinterp();
  CV._interpolation( coefmon, std::fabs );

  double b(Op<T>::mid(CV.B())), a(Op<T>::u(CV.B())-b), rem, ub(0), lb(0);
  for (unsigned i(0); i<=CV.nord(); i++) {
    ub += coefmon[i];
    lb += std::pow(-1.,i)*coefmon[i];
    }
  rem = std::max(std::fabs(a+b)-ub, std::fabs(b-a)-lb);

  CVI = CV._rescale(a,b);
  CV2 = CVI._composition( coefmon );
  //CV2 += T(-rem, rem);
  CV2 += (2.*Op<T>::zeroone()-1.)*rem;

  //if( CV._CM->options.CENTER_REMAINDER ) CV2._center();
  return CV2;
*/
}

template <typename T> inline CVar<T>
hull
( const CVar<T>&CV1, const CVar<T>&CV2 )
{
  // Neither operands associated to CModel -- Make intersection in T type     
  if( !CV1._CM && !CV2._CM ){
    T R1 = CV1._coefmon[0] + *(CV1._bndrem);
    T R2 = CV2._coefmon[0] + *(CV2._bndrem);
    return Op<T>::hull(R1, R2);
  }

  // First operand not associated to CModel
  else if( !CV1._CM )
    return hull( CV2, CV1 );

  // Second operand not associated to CModel
  else if( !CV2._CM ){
    CVar<T> CVR = CV1.P();
    return CVR + Op<T>::hull( CV1.R(), CV2._coefmon[0]+*(CV2._bndrem)-CVR.B() );
  }

  // CModel for first and second operands are inconsistent
  else if( CV1._CM != CV2._CM )
    throw typename CModel<T>::Exceptions( CModel<T>::Exceptions::CMODEL );

  // Perform union
  CVar<T> CV1C( CV1 ), CV2C( CV2 );
  const double eta = CV1._CM->options.REF_POLY;
  T R1C = CV1C.C().R(), R2C = CV2C.C().R(); 
  CV1C.set(T(0.));
  CV2C.set(T(0.));
  T BCVD = (CV1C-CV2C).B();
  return (1.-eta)*CV1C + eta*CV2C + Op<T>::hull( R1C+eta*BCVD, R2C+(eta-1.)*BCVD );
}

template <typename T> inline bool
inter
( CVar<T>&CVR, const CVar<T>&CV1, const CVar<T>&CV2 )
{
  // Neither operands associated to CModel -- Make intersection in T type     
  if( !CV1._CM && !CV2._CM ){
    T R1 = CV1._coefmon[0] + CV1._bndord[0];
    T R2 = CV2._coefmon[0] + CV2._bndord[0];
    T RR( 0. );
    bool flag = Op<T>::inter(RR, R1, R2);
    CVR = RR;
    return flag;
  }

  // First operand not associated to CModel
  else if( !CV1._CM )
    return inter( CVR, CV2, CV1 );

  // Second operand not associated to CModel
  else if( !CV2._CM ){
    // First intersect in T arithmetic
    T B2 = CV2.B(), BR;
    if( CV1._CM->options.MIXED_IA && !Op<T>::inter( BR, CV1.B(), B2 ) )
      return false;

    // Perform intersection in PM arithmetic
    T R1 = CV1.R();
    CVR = CV1.P();
    if( !Op<T>::inter(*(CVR._bndrem), R1, B2-CVR.B()) )
      return false;
    CVR._center();

    if( CVR._CM->options.MIXED_IA ) CVR._set_bndT( BR );
    else CVR._unset_bndT();
    return true;
  }

  // CModel for first and second operands are inconsistent
  else if( CV1._CM != CV2._CM )
    throw typename CModel<T>::Exceptions( CModel<T>::Exceptions::CMODEL );

  // First intersect in T arithmetic
  T BR;
  if( CV1._CM->options.MIXED_IA && !Op<T>::inter( BR, CV1.B(), CV2.B() ) )
    return false;

  // Perform intersection in PM arithmetic
  CVar<T> CV1C( CV1 ), CV2C( CV2 );
  const double eta = CV1._CM->options.REF_POLY;
  T R1C = CV1C.C().R(), R2C = CV2C.C().R(); 
  CV1C.set(T(0.));
  CV2C.set(T(0.));
  CVR = (1.-eta)*CV1C + eta*CV2C;
  CV1C -= CV2C;
  T BCVD = CV1C.B();
  if( !Op<T>::inter( *(CVR._bndrem), R1C+eta*BCVD, R2C+(eta-1.)*BCVD ) )
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

//! @brief Specialization of the structure mc::Op to allow usage of the type mc::Interval for DAG evaluation or as a template parameter in other MC++ classes
template<typename T> struct Op< mc::CVar<T> >
{
  typedef mc::CVar<T> CV;
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
  static CV erf (const CV& x) { throw typename mc::CModel<T>::Exceptions( CModel<T>::Exceptions::UNDEF ); }
  static CV erfc(const CV& x) { throw typename mc::CModel<T>::Exceptions( CModel<T>::Exceptions::UNDEF ); }
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

