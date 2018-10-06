// Copyright (C) 2009-2016 Benoit Chachuat, Imperial College London.
// All Rights Reserved.
// This code is published under the Eclipse Public License.

#ifndef MC__POLYMODEL_H
#define MC__POLYMODEL_H

#include <iostream>
#include <iomanip>
#include <typeinfo>
#include <sstream>
#include <string>
#include <vector>
#include <set>
#include <map>
#include <stdarg.h>
#include <cassert>
#include <climits>
#include <limits>
#include <stdlib.h>
#include <complex>

#include "mcfunc.hpp"
#include "mclapack.hpp"
#include "mcop.hpp"

#undef  MC__POLYMODEL_DEBUG
#define MC__POLYMODEL_DEBUG_POLYBOUND
#define MC__POLYMODEL_CHECK

namespace mc
{

typedef unsigned long long poly_size;

//! @brief C++ base class for the computation of polynomial models for factorable functions: Environment
////////////////////////////////////////////////////////////////////////
//! mc::PolyModel is a C++ base class for definition of polynomial model
//! environment.
////////////////////////////////////////////////////////////////////////
class PolyModel
////////////////////////////////////////////////////////////////////////
{
public:

  /** @addtogroup POLYMOD Polynomial Model Arithmetic for Factorable Functions
   *  @{
   */
  //! @brief Constructor of polynomial model environment for <tt>nvar</tt> variables and order <tt>nord</tt>
  PolyModel
    ( const unsigned nvar, const unsigned nord, const bool sparse=false )
    { _size( nvar, nord, sparse ); }
  //! @brief Destructor of polynomial model environment

  virtual ~PolyModel
    ()
    { _cleanup(); }

  //! @brief Whether or not sparse representation is used
  unsigned sparse
    ()
    const
    { return _sparse; };

  //! @brief Number of variables in polynomial model environment
  unsigned nvar
    ()
    const
    { return _nvar; };

  //! @brief Order of polynomial model environment
  unsigned nord
    ()
    const
    { return _nord; };

  //! @brief Total number of monomial terms in polynomial model
  unsigned nmon
    ()
    const
    { return _nmon; };

  //! @brief Const pointer to array of size <tt>nmon()*nvar()</tt> with variable exponents on each monomial term. The exponent for variable <tt>ivar</tt> in monomial term <tt>imon</tt> is at position <tt>imon*nvar()+ivar</tt>.
  const unsigned* expmon
    ()
    const
    { return _expmon; };

  //! @brief Const pointer to array of size <a>nvar</a> of variable exponents in monomial term <tt>imon</tt> of polynomial model
  const unsigned* expmon
    ( const unsigned imon )
    const
    { return _expmon+imon*_nvar; }

  //! @brief Index of monomial term whose variable exponents are the same as those in array <tt>iexp</tt> (of size <tt>nvar()</tt>)
  unsigned loc_expmon
    ( const unsigned *iexp )
    const
    { return _loc_expmon( iexp ); }

  //! @brief Array of size <tt>_nord</tt> with indices of first monomial term of order <tt>iord=1,...,_nord</tt> in polynomial model
  const unsigned* posord
    ()
    const
    { return _posord; }

  //! @brief Get binomial coefficient \f$\left(\stackrel{n}{k}\right)\f$
  poly_size get_binom
    ( const unsigned n, const unsigned k )
    const
    { return _get_binom( n, k ); }

  //! @brief Get const pointer to array of size <tt>nvar</tt> with references for all variables in Taylor model
  const double* refvar() const
    { return _refvar; }

  //! @brief Get const pointer to array of size <tt>nvar</tt> with scaling coefficients in Taylor model
  const double* scalvar() const
    { return _scalvar; }

  //! @brief Exceptions of mc::PolyModel
  class Exceptions
  {
  public:
    //! @brief Enumeration type for PolyModel exception handling
    enum TYPE{
      SIZE=1,	//!< Number of variable in polynomial model must be nonzero
      MAXSIZE,  //!< Maximum size of polynomial model reached (monomials indexed as unsigned)
      UNDEF=-33 //!< Feature not yet implemented in mc::PolyModel
    };
    //! @brief Constructor for error <a>ierr</a>
    Exceptions( TYPE ierr ) : _ierr( ierr ){}
    //! @brief Error flag
    int ierr(){ return _ierr; }
    //! @brief Error description
    std::string what(){
      switch( _ierr ){
      case SIZE:
        return "mc::PolyModel\t Inconsistent polynomial model dimension";
      case MAXSIZE:
        return "mc::PolyModel\t Maximum size in polynomial model reached";
      case UNDEF:
        return "mc::PolyModel\t Feature not yet implemented in mc::PolyModel class";
      default:
        return "mc::PolyModel\t Undocumented error";
      }
    }

  private:
    TYPE _ierr;
  };
  /** @} */

private:  
  //! @brief Set polynomial model order <tt>nord</tt> and number of variables <tt>nvar</tt>
  void _size
    ( const unsigned nvar, const unsigned nord, const bool sparse );

  //! @brief Clean up internal arrays
  void _cleanup();

protected:
  //! @brief Flag indicating whether to use sparse computation where suitable
  bool _sparse;

  //! @brief Order of polynomial model
  unsigned _nord;

  //! @brief Number of variables in polynomial model
  unsigned _nvar;

  //! @brief Total number of monomial terms in polynomial model
  unsigned _nmon;

  //! @brief Array of size <tt>_nord</tt> with indices of first monomial term of order <tt>iord=1,...,_nord</tt> in polynomial model
  unsigned *_posord;

  //! @brief Order used to size _posord
  unsigned _posord_size;

  //! @brief Array of size <tt>_nmon*_nvar</tt> with variable exponents in monomial terms. The exponent for variable <tt>ivar</tt> in monomial term <tt>imon</tt> is at location <tt>imon*nvar()+ivar</tt>.
  unsigned *_expmon;

  //! @brief Number of monomial coefficients used to size _expmon
  unsigned _expmon_size;

  //! @brief Array of <tt>(_nvar+_nord-1)*(_nord+1)</tt> contining binomial coefficients
  poly_size *_binom;

  //! @brief Maximum binomial coefficients in array _binom
  std::pair<unsigned, unsigned> _binom_size;

  //! @brief Array of size <tt>_nvar</tt> with reference points for the variables
  double *_refvar;

  //! @brief Array of size <tt>_nvar</tt> with scaling for the variables
  double *_scalvar; 

  //! @brief Have any of the model variables been modified?
  bool _modvar;

  //! @brief Populate array <tt>_posord</tt> up to order <tt>nord</tt>
  void _set_posord
    ( const unsigned nord );

  //! @brief Extend array <tt>_posord</tt> up to maximum order <tt>maxord</tt>
  void _ext_posord
    ( const unsigned maxord );

  //! @brief Populate array <tt>_expmon</tt> up to order <tt>nord</tt>
  void _set_expmon
    ( const unsigned nord );

  //! @brief Extend array <tt>_expmon</tt> up to order <tt>maxord</tt> to accomodate <tt>maxmon</tt> coefficients
  void _ext_expmon
    ( const unsigned maxord, const bool full=false );
  
  //! @brief Generate variable exponents <tt>iexp</tt> for subsequent monomial order <tt>iord</tt>
  void _next_expmon
    ( unsigned *iexp, const unsigned iord ) const;

  //! @brief Get index of monomial term with variable exponents <tt>iexp</tt> in <tt>1,...,_nmon</tt>
  unsigned _loc_expmon
    ( const unsigned *iexp ) const;
    
  //! @brief Populate array <tt>_binom</tt> with binomial coefficients up to order <tt>nord</tt>
  void _set_binom
    ( const unsigned nord );
    
  //! @brief Extend array <tt>_binom</tt> with binomial coefficients up to order <tt>nord</tt>
  void _ext_binom
    ( const unsigned nord );

  //! @brief Get binomial coefficient \f$\left(\stackrel{n}{k}\right)\f$
  poly_size _get_binom
    ( const unsigned n, const unsigned k ) const;

  //! @brief Compute bounds for all terms of degrees <tt>iord=0,...,_nord</tt> given the Chebyshev basis function in <tt>bndbasis</tt>
  template <typename C, typename U> U* _get_bndord
    ( const C*coefmon, const U*const*bndbasis,
      const std::set<unsigned>&ndxmon=std::set<unsigned>() ) const;

  //! @brief Polynomial range bounder - naive approach
  template <typename C, typename U> U _polybound_naive
    ( const C*coefmon, const U*bndord,
      const std::set<unsigned>&ndxmon=std::set<unsigned>() ) const;
};

//////////////////////////////// PolyModel ////////////////////////////////////

inline void
PolyModel::_size
( const unsigned nvar, const unsigned nord, const bool sparse )
{
  if( !nvar ) throw Exceptions( Exceptions::SIZE );

  _nvar = nvar;
  _nord = nord;
  _sparse = sparse;

  _binom = new poly_size[nord?(nvar+nord-1)*(nord+1):nvar];
  _binom_size = std::make_pair( nvar+nord-1, nord+1 );
  _set_binom( nord );
  _posord = new unsigned[nord+2];
  _posord_size = nord;
  _set_posord( nord );
  _nmon = _posord[_nord+1];
  _expmon = new unsigned[_nmon*nvar];
  _expmon_size = _nmon;
  _set_expmon( nord );

  _refvar = new double[_nvar];
  _scalvar  = new double[_nvar];
  _modvar = true;
}

inline void
PolyModel::_cleanup()
{
  delete[] _expmon;
  delete[] _posord;
  delete[] _binom;
  delete[] _refvar;
  delete[] _scalvar;
}

inline void
PolyModel::_set_posord
( const unsigned nord )
{
  _posord[0] = 0;
  _posord[1] = 1;
  for( unsigned i=1; i<=nord; i++ ){
    poly_size _posord_next = _posord[i] + _get_binom( _nvar+i-1, i );
    if( _posord_next > UINT_MAX )
      throw typename PolyModel::Exceptions( PolyModel::Exceptions::MAXSIZE );
    _posord[i+1] = _posord_next;
  }

#ifdef MC__POLYMODEL_DEBUG
  mc::display( 1, nord+2, _posord, 1, "_posord", std::cout );
#endif
}
    
inline void
PolyModel::_ext_posord
( const unsigned maxord )
{
  if( maxord < _posord_size ) return;
  delete[] _posord;
  _posord = new unsigned[maxord+2];
  _posord_size = maxord;
  _set_posord( maxord );
}

inline void
PolyModel::_set_expmon
( const unsigned nord )
{
  unsigned *iexp = new unsigned[_nvar] ;
  for( unsigned k=0; k<_nvar; k++ ) _expmon[k] = iexp[k] = 0;
  for( unsigned i=1; i<=nord; i++ ){
    //for( unsigned j=0; j<_nvar; j++ ) iexp[j] = 0;
    for( unsigned j=_posord[i]; j<_posord[i+1]; j++ ){
      _next_expmon( iexp, _nvar );
      for( unsigned k=0; k<_nvar; k++ )
        _expmon[j*_nvar+k] = iexp[k];
    }
  }
  delete[] iexp;

#ifdef MC__POLYMODEL_DEBUG
  mc::display( _nvar, _expmon_size, _expmon, _nvar, "_expmon", std::cout );
#endif
}

// Based on a code by John Burkardt
// http://people.sc.fsu.edu/~jburkardt/cpp_src/monomial/monomial.html
inline void
PolyModel::_next_expmon
( unsigned *x, const unsigned m ) const
{
  // Ensure that M>=1
  assert( m );

  // Find I, the index of the rightmost nonzero entry of X.
  unsigned i=0, im1, t;
  for( unsigned j=m; j>=1; j-- ){
    if ( x[j-1] ){
      i = j;
      break;
    }
  }

  // set T = X(I)
  // set X(I) to zero,
  // increase X(I-1) by 1,
  // increment X(D) by T-1.
  if( !i ){
    x[m-1] = 1;
    return;
  }
  else if( i == 1 ){
    t = x[0] + 1;
    im1 = m;
  }
  else{
    t = x[i-1];
    im1 = i - 1;
  }

  x[i-1] = 0;
  x[im1-1] = x[im1-1] + 1;
  x[m-1] = x[m-1] + t - 1;
}
/*
inline void
PolyModel::_next_expmon
( unsigned *iexp, const unsigned iord ) const
{
  unsigned curord;
  do{
    iexp[_nvar-1] += iord;
    unsigned j = _nvar;
    while( j > 0 && iexp[j-1] > iord ){
      iexp[j-1] -= iord + 1;
      j-- ;
      iexp[j-1]++;
    }
    curord = 0;
    for( unsigned i=0; i<_nvar; i++ ) curord += iexp[i];
  } while( curord != iord );
}
*/  
inline void
PolyModel::_ext_expmon
( const unsigned maxord, const bool full )
{
  _ext_binom( maxord );
  _ext_posord( maxord ); 
  delete[] _expmon;
  if( full ) _expmon_size = std::pow(maxord+1,_nvar);
  else       _expmon_size = _posord[maxord+1];
  _expmon = new unsigned[ _expmon_size*_nvar];
  _set_expmon( maxord );
  if( !full ) return;

  unsigned *iexp = new unsigned[_nvar];
  for( unsigned iord=maxord+1, jmon=_posord[maxord+1];
       jmon<_expmon_size; iord++ ){
    _ext_binom( iord );
    _ext_posord( iord );
    if( _posord[iord] >= _posord[iord+1] )
      throw typename PolyModel::Exceptions( PolyModel::Exceptions::MAXSIZE );
    for( unsigned ivar=0; ivar<_nvar; ivar++ ) iexp[ivar] = _expmon[(_nmon-1)*_nvar+ivar];//0;
    for( unsigned kmon=_posord[iord]; kmon<_posord[iord+1]; kmon++ ){
      _next_expmon( iexp, _nvar );//iord );
      bool lt_maxord = true;
      for( unsigned ivar=0; ivar<_nvar; ivar++ ){
        if( iexp[ivar] > maxord ){ lt_maxord = false; break; }
        _expmon[jmon*_nvar+ivar] = iexp[ivar];
      }
      if( lt_maxord ){
#ifdef MC__POLYMODEL_DEBUG
	std::cout << jmon << ":";
        for( unsigned ivar=0; ivar<_nvar; ivar++ )
	  std::cout << "  " << _expmon[jmon*_nvar+ivar];
	std::cout << std::endl;
#endif
        jmon++;
        if( jmon >= _expmon_size ) break;
      }
    }
  }
  delete[] iexp;
  return;
}
  
inline unsigned
PolyModel::_loc_expmon
( const unsigned *iexp ) const
{
  unsigned ord = 0;
  for( unsigned i=0; i<_nvar; i++ ) ord += iexp[i];
#ifdef MC__POLYMODEL_CHECK
  assert( ord<_nord+2 );
#endif
  unsigned pos = _posord[ord];
  
  unsigned p = _nvar ; 
  for( unsigned i=0; i<_nvar-1; i++ ){
    p--;
    for( unsigned j=0; j<iexp[i]; j++ )
      pos += _get_binom( p-1+ord-j, ord-j );
    ord -= iexp[i];
  }

  return pos;    
}
 
inline void
PolyModel::_set_binom
( const unsigned nord )
{
  poly_size *p;
  unsigned k;
  for( unsigned i=0; i<_nvar+nord-1; i++ ){
    p = &_binom[i*(nord+1)];
    *p = 1;
    p++;
    *p = i+1;
    p++;
    k = ( i+1<nord? i+1: nord );
    for( unsigned j=2; j<=k; j++, p++ ) *p = *(p-1) * (i+2-j)/j;
    for( unsigned j=k+1; j<=nord; j++, p++ ) *p = 0.;
  }
#ifdef MC__POLYMODEL_DEBUG
  std::cout << "nvar: " << _nvar << "  nord: " << nord << std::endl;
  mc::display( _binom_size.second, _binom_size.first, _binom,
    _binom_size.second, "_binom", std::cout );
#endif
}
    
inline void
PolyModel::_ext_binom
( const unsigned maxord )
{
  if( maxord < _binom_size.second ) return;
  delete[] _binom;
  _binom = new poly_size[(_nvar+maxord-1)*(maxord+1)];
  _binom_size = std::make_pair( _nvar+maxord-1, maxord+1 );
  _set_binom( maxord );
}

inline poly_size
PolyModel::_get_binom
( const unsigned n, const unsigned k ) const
{
#ifdef MC__POLYMODEL_CHECK
  assert( n<=_binom_size.first );
  assert( k<=_binom_size.second );
  assert( k<=n );
#endif
  return( n? _binom[(n-1)*_binom_size.second+k]: 1 );
}

template <typename C, typename U> inline U*
PolyModel::_get_bndord
( const C*coefmon, const U*const*bndbasis,
  const std::set<unsigned>&ndxmon ) const
{
  U* bndord_ = new U[_nord+1];
  bndord_[0] = (ndxmon.empty() || ndxmon.find(0)!=ndxmon.end())? coefmon[0]: 0.;
  //bndord_[0] = coefmon[0];
  for( unsigned i=1; i<=_nord; i++ ){
    bndord_[i] = 0.;
    for( auto jt=ndxmon.lower_bound(_posord[i]); jt!=ndxmon.lower_bound(_posord[i+1]); ++jt ){
      unsigned k=0;
      const unsigned *jexp = _expmon+(*jt)*_nvar;
      for( ; k<_nvar; k++ ) if( jexp[k] ) break;
      U bndmon_ = bndbasis[k][jexp[k]];
      for( ++k; k<_nvar; k++ ) if( jexp[k] ) bndmon_ *= bndbasis[k][jexp[k]];
      bndord_[i] += coefmon[*jt] * bndmon_;
    }
    for( unsigned j=_posord[i]; ndxmon.empty() && j<_posord[i+1]; j++ ){
      unsigned k=0;
      const unsigned *jexp = _expmon+j*_nvar;
      for( ; k<_nvar; k++ ) if( jexp[k] ) break;
      U bndmon_ = bndbasis[k][jexp[k]];
      for( ++k; k<_nvar; k++ ) if( jexp[k] ) bndmon_ *= bndbasis[k][jexp[k]];
      bndord_[i] += coefmon[j] * bndmon_;
    }
  }
  return bndord_;
}

template <typename C, typename U> inline U
PolyModel::_polybound_naive
( const C*coefmon, const U*bndord,
  const std::set<unsigned>&ndxmon ) const
{
  U bndpol = (ndxmon.empty() || ndxmon.find(0)!=ndxmon.end())? coefmon[0]: 0.;
  //U bndpol = coefmon[0];
  for( unsigned i=1; i<=_nord; i++ ) bndpol += bndord[i];
  return bndpol;
}

//! @brief C++ base class for the computation of polynomial models for factorable functions: Variable
////////////////////////////////////////////////////////////////////////
//! mc::PolyVar is a C++ base class for definition of polynomial model
//! variables.
////////////////////////////////////////////////////////////////////////
template <typename T>
class PolyVar
////////////////////////////////////////////////////////////////////////
{

protected:
  //! @brief Pointer to corresponding polynomial model environment
  PolyModel* _PM;

  //! @brief Array of size <tt>_nmon</tt> with monomial coefficients of variable
  double* _coefmon;

  //! @brief Array of size <tt>_nord+2</tt> with bounds for all terms of degrees <tt>iord=0,...,_nord</tt> as well as the remainder bound at position <tt>_nord+1</tt> of variable (possibly inaccurate - see _bndord_uptd)
  mutable T* _bndord; 

  //! @brief Whether the bounds in bndord are up-to-date
  mutable bool _bndord_uptd;

  //! @brief Pointer to remainder bound of variable (possibly NULL if not computed)
  T* _bndrem;

  //! @brief Pointer to variable bound in underlying T arithmetic (possibly NULL if not computed)
  mutable T* _bndT;

  //! @brief Pointer to polynomial bound of variable
  mutable T* _bndpol;

  //! @brief Indices of nonzero terms in sparse representation
  std::set<unsigned> _ndxmon;

public:
  //! @brief Order of polynomial model
  unsigned nord
    ()
    const
    { return( _PM? _PM->nord(): 0 ); };

  //! @brief Number of variables in polynomial model
  unsigned nvar
    ()
    const
    { return( _PM? _PM->nvar(): 0 ); };

  //! @brief Total number of monomial terms in polynomial model
  unsigned nmon
    ()
    const
    { return( _PM? _PM->nmon(): 1 ); };

protected:
  //! @brief Index of first monomial term of order <a>iord</a> in polynomial model
  unsigned _posord
    ( const unsigned iord )
    const
    { return _PM->posord()[iord]; };

  //! @brief Const pointer to array of size <a>nvar</a> of variable exponents in monomial term <tt>imon</tt> of polynomial model
  const unsigned* _expmon
    ( const unsigned imon )
    const
    { return _PM->expmon(imon); };

  //! @brief Index of monomial term whose variable exponents are the same as those in array <tt>iexp</tt> (of size <tt>nvar()</tt>)
  const unsigned _loc_expmon
    ( const unsigned* iexp )
    const
    { return _PM->loc_expmon( iexp ); };

  //! @brief Get binomial coefficient \f$\left(\stackrel{n}{k}\right)\f$
  poly_size _get_binom
    ( const unsigned n, const unsigned k )
    const
    { return _PM->get_binom( n, k ); }

  //! @brief Initialize private/protected members of variable
  void _size
    ( PolyModel* env );

  //! @brief Reinitialize private/protected members of variable
  void _resize
    ( PolyModel* env, const bool init=false );

  //! @brief Clean up private/protected members of variable
  void _cleanup
    ();

  //! @brief Set variable equal to <a>var</a>
  PolyVar<T>& _set
    ( const PolyVar<T>&var );

  //! @brief Center remainder error term <tt>_bndrem</tt>
  virtual void _center();

  //! @brief Set variable bound in unerlying T arithmetic
  virtual void _set_bndT
    ( const T&bndT );

  //! @brief Set variable bound in unerlying T arithmetic
  virtual void _set_bndT
    ( const T*bndT );

  //! @brief Unset variable bound in underlying T arithmetic
  virtual void _unset_bndT
    ();

  //! @brief Set polynomial bound in variable as <tt>bndpol</tt>
  virtual void _set_bndpol
    ( const T&bndpol );

  //! @brief Set polynomial bound in variable as <tt>bndpol</tt>
  virtual void _set_bndpol
    ( const T*bndpol );

  //! @brief Unset polynomial bound in variable
  virtual void _unset_bndpol
    ();

  //! @brief Polynomial range bounder using specified bounder <a>type</a> (pure virtual)
  virtual T _polybound
    ( const int type )
    const
    = 0;
 
  //! @brief Polynomial range bounder using default bounder (pure virtual)
  virtual T _polybound
    ()
    const
    = 0;
 
  //! @brief Update private/protected members of variable after manipulation the coefficients (pure virtual)
  virtual void _update_bndord
    () const
    = 0;

public:
  /** @addtogroup POLYMOD Polynomial Model Arithmetic for Factorable Functions
   *  @{
   */
  //! @brief Constructor of variable linked to polynomial model environment <a>env</a>
  PolyVar
    ( PolyModel* env=0 )
    { _size( env ); }

  //! @brief Copy constructor of variable
  PolyVar
    ( const PolyVar<T>& var )
    { _size( var._PM );
      // Set variable not linked to any environment
      if( !_PM ){
        _coefmon[0] = var._coefmon[0];
        _bndord[0]  = var._bndord[0];
      }
      // Set variable linked to an environment
      else{
        for( unsigned i=0; i<nmon(); i++ ) _coefmon[i] = var._coefmon[i];
        for( unsigned i=0; i<nord()+2; i++) _bndord[i] = var._bndord[i];
      }
      _bndord_uptd = var._bndord_uptd;
      if( var._bndpol ) _bndpol = new T( *var._bndpol );
      if( var._bndT )   _bndT   = new T( *var._bndT   );

      // Copy sparsity information
      _ndxmon = var._ndxmon;
    }

  //! @brief Destructor of variable
  virtual ~PolyVar()
    { delete[] _coefmon; delete[] _bndord; delete _bndpol; delete _bndT; }

  //! @brief Set polynomial model environment in variable as <tt>env</tt>
  virtual PolyVar<T>& set
    ( PolyModel* env, const bool reset=false )
    { _resize( env, reset );
      return *this; }
    //{ _PM = env; return *this; }

  //! @brief Set multivariate polynomial coefficients in variable as <tt>coefmon</tt>
  virtual PolyVar<T>& set
    ( const double* coefmon )
    { for( unsigned imon=0; imon<(_PM?nmon():1); imon++ )
        _coefmon[imon] = coefmon[imon];
      _unset_bndT(); _unset_bndpol(); _bndord_uptd = false;
      return *this; }

  //! @brief Set multivariate polynomial coefficients in variable as <tt>coefmon</tt> - only first <tt>coefmon.first</tt> coefficients are set
  virtual PolyVar<T>& set
    ( std::pair<unsigned, const double*>& coefmon )
    { for( unsigned imon=0; imon<(_PM?nmon():1); imon++ )
        _coefmon[imon] = ( imon<coefmon.first && coefmon.second?
	                   coefmon.second[imon]: 0. );
      _unset_bndT(); _unset_bndpol(); _bndord_uptd = false;
      return *this; }

  //! @brief Set remainder term in variable as <tt>bndrem</tt>
  virtual PolyVar<T>& set
    ( const T& bndrem )
    { *_bndrem = bndrem;
      return *this; }

  //! @brief Set multivariate polynomial coefficients and remainder term equal to those in variable <tt>var</tt>, possibly defined in another Taylor model environment with less variables or with a different expansion order. Coefficients involving other variables or higher order are initialized to 0 if <tt>reset=true</tt> (default), otherwise they are left unmodified. Higher-order terms in TV are bounded and added to the remainder bound.
  virtual PolyVar<T>& set
    ( const PolyVar<T>& var, const bool reset=true );

  //! @brief Copy multivariate polynomial coefficients from current variable into variable <tt>var</tt>, possibly defined in another polynomial model environment with less variables or with a lower expansion order. Copied coefficients are reset to 0 in current Taylor variable if <tt>reset=true</tt>, otherwise they are left unmodified (default).
  virtual PolyVar<T>& get
    ( PolyVar<T>&var, const bool reset=false );

  //! @brief Get pointer to associated polynomial model environment
  virtual PolyModel* env() const
    { return _PM; }

  //! @brief Compute bound on variable using bounder <a>type</a>
  virtual T bound
    ( const int type ) const
    { if( !_bndT ) return _polybound(type) + *_bndrem;
      else{
        T bndT; return Op<T>::inter( bndT, _polybound(type) + *_bndrem, *_bndT )?
            bndT: _polybound(type) + *_bndrem;
      }
    }

  //! @brief Retreive bound on variable using default bounder
  T bound() const
    { if( !_bndT ) return bndpol() + *_bndrem;
      else{
        T bndT; return Op<T>::inter( bndT, bndpol() + *_bndrem, *_bndT )?
            bndT: bndpol() + *_bndrem;
      }
    }

  //! @brief Retreive bound on multivariate polynomial using default bounder
  T bndpol() const
    { if( !_bndpol ) _bndpol = new T( _polybound() );
      return *_bndpol; }

  //! @brief Retreive bound on all terms with (total) order <tt>iord</tt> in polynomial model
  T bndord
    ( const unsigned iord )
    const
    { if( !iord || (_PM && iord<=nord()) )
      { if( !_bndord_uptd ) _update_bndord(); return _bndord[iord]; }
      return 0.; }
    //{ return ( !iord || (_PM && iord<=nord()) )? _bndord[iord]: 0.; }

  //! @brief Return remainder term of variable
  T remainder
    ()
    const
    { return( *_bndrem ); }

  //! @brief Shortcut to mc::PolyVar::bound
  virtual T B
    ( const int type )
    const
    { return bound( type ); }

  //! @brief Shortcut to mc::PolyVar::bound
  virtual T B
    ()
    const
    { return bound(); }

  //! @brief Shortcut to mc::PolyVar::remainder
  T R
    ()
    const
    { return remainder(); }

  //! @brief Get (possibly scaled) coefficient in monomial term with variable exponents as given in <a>iexp</a>
  double coefmon
    ( const unsigned*iexp )
    const;

  //! @brief Get (possibly scaled) coefficient in monomial term with variable exponents as given in <a>iexp</a>
  double& coefmon
    ( const unsigned*iexp );

  //! @brief Get pair of size of, and const pointer to, array of (possibly scaled) monomial coefficients
  std::pair<unsigned, const double*> coefmon
    ()
    const;

  //! @brief Get pair of size of, and const pointer to, array of monomial exponents
  std::pair<unsigned, const unsigned*> expmon
    ()
    const;

  //! @brief Get const pointer to kth monomial exponents
  const unsigned* expmon
    ( const unsigned k )
    const;

  //! @brief Get set of monomial indices in sparse representation
  const std::set<unsigned>& ndxmon
    ()
    const;

  //! @brief Overloaded operator '=' for polynomial model variables
  virtual PolyVar<T>& operator=
    ( const PolyVar<T>& );
  /** @} */
};

///////////////////////////////// PolyVar /////////////////////////////////////

template <typename T> inline void
PolyVar<T>::_size
( PolyModel* env )
{
  _PM = env;
  if( !_PM ){
    _coefmon = new double[1];
    _bndord  = new T[1];
    _bndrem  = _bndord;
  }
  else{
    _coefmon = new double[nmon()];
    _bndord  = new T[nord()+2];
    _bndrem  = _bndord + nord()+1;
  }
  _bndord_uptd = false;
  _bndpol = 0;
  _bndT = 0;
  return;
}

template <typename T> inline void
PolyVar<T>::_cleanup()
{
  delete [] _coefmon; delete [] _bndord; delete _bndpol; delete _bndT;
  _coefmon = 0; _bndord = _bndrem = _bndpol = _bndT = 0;
  _ndxmon.clear();
}

template <typename T> inline void
PolyVar<T>::_resize
( PolyModel* env, const bool init )
{
  if( _PM == env ) return;
  _cleanup();
  _size( env );
  if( init ){
    for( unsigned k=0; k<nmon(); k++ ) _coefmon[k] = 0.;
    *_bndrem = 0.;
  }

}

template <typename T> inline PolyVar<T>&
PolyVar<T>::operator=
( const PolyVar<T>&var )
{
  return _set( var );
}

template <typename T> inline PolyVar<T>&
PolyVar<T>::_set
( const PolyVar<T>&var )
{
  // Same PolyVar?
  if( this == &var ) return *this;

  // Reinitialization needed?
  _resize( var._PM, !var._ndxmon.empty() );

  // Set variable not linked to any environment
  if( !_PM ){
    _coefmon[0] = var._coefmon[0];
    *_bndrem    = *var._bndrem;
  }

  // Set variable linked to an environment
  else{
    _ndxmon.clear();
    // Case: sparse representation
    for( auto it=var._ndxmon.begin(); it!=var._ndxmon.end(); ++it ){
      _ndxmon.insert(*it);
      _coefmon[*it] = var._coefmon[*it];
    }
    // Case: dense representation
    for( unsigned jmon=0; var._ndxmon.empty() && jmon<nmon(); jmon++ )
      _coefmon[jmon] = var._coefmon[jmon];
    // Set order and remainder bounds
    for( unsigned i=0; var._bndord_uptd && i<nord()+1; i++)
      _bndord[i] = var._bndord[i];
    *_bndrem = *var._bndrem;
  }
  _bndord_uptd = var._bndord_uptd; 

  // Set polynomial bound
  _set_bndpol( var._bndpol );
  // Set underlying variable bound
  _set_bndT( var._bndT );

  return *this;
}

template <typename T> inline PolyVar<T>&
PolyVar<T>::set
( const PolyVar<T>& var, const bool reset )
{
  if( !_PM || ( var._PM && nvar() < var.nvar() ) ) return *this;

  // Reset monomial coefficients and remainder
  if( reset ){
    for( unsigned imon=0; imon<nmon(); imon++ ) _coefmon[imon] = 0.;
    *_bndrem = 0.;
    _ndxmon.clear();
  }

  // Copy monomial coefficients from var into *this
  if( !var._PM ){
    _coefmon[0] = var._coefmon[0];
    *_bndrem = *var._bndrem;
    _bndord_uptd = false;
    return *this;
  }
  unsigned*iexp = new unsigned[nvar()];
  // Case: sparse representation
  for( auto it=var._ndxmon.begin();
       it!=var._ndxmon.end() && *it<var._posord(nord()+1); ++it ){
    for( unsigned ivar=0; ivar<nvar(); ivar++ )
      iexp[ivar] = ( ivar<var.nvar()? var._expmon(*it)[ivar]: 0 );
    //unsigned imon = _loc_expmon(iexp);
    auto pmon = _ndxmon.insert( _loc_expmon(iexp) );
    _coefmon[*(pmon.first)] = var._coefmon[*it];
  }
  // Case: dense representation
  for( unsigned jmon=0;
       _ndxmon.empty() && jmon<var.nmon() && jmon<var._posord(nord()+1); jmon++ ){
    for( unsigned ivar=0; ivar<nvar(); ivar++ )
      iexp[ivar] = ( ivar<var.nvar()? var._expmon(jmon)[ivar]: 0 );
    _coefmon[_loc_expmon(iexp)] = var._coefmon[jmon];
  }
  delete[] iexp;
  for( unsigned iord=nord()+1; iord<=var.nord(); iord++ )
    *_bndrem += var.bndord(iord);
  *_bndrem += *var._bndrem;
  _unset_bndT();
  _unset_bndpol();
  _bndord_uptd = false;

  return *this;
}

template <typename T> inline PolyVar<T>&
PolyVar<T>::get
( PolyVar<T>& var, const bool reset )
{
  if( !_PM ){
    var._ndxmon.clear();
    if( var._PM && var._PM->sparse() ) var._ndxmon.insert(0);
    var._coefmon[0] = _coefmon[0];
    // Reset monomial coefficients to 0
    if( reset ) _coefmon[0] = 0.;
    return *this;
  }
  if( !var._PM || nvar() < var.nvar() || nord() < var.nord() ) // looks fishy...
    return *this;

  // Copy monomial coefficients from *this into var
  unsigned*iexp = new unsigned[nvar()];
  // Case: sparse representation
  for( auto it=_ndxmon.begin(); it!=_ndxmon.end() && *it<_posord(nord()+1); ++it ){
    for( unsigned ivar=0; ivar<nvar(); ivar++ )
      iexp[ivar] = ( ivar<var.nvar()? var._expmon(*it)[ivar]: 0 );
    //unsigned imon = _loc_expmon(iexp);
    auto pmon = _ndxmon.insert( _loc_expmon(iexp) );
    var._coefmon[*it] = _coefmon[*(pmon.first)];
    // Reset monomial coefficients to 0
    if( reset ) _coefmon[*(pmon.first)] = 0.;
  }
  // Case: dense representation
  for( unsigned jmon=0; _ndxmon.empty() && jmon<var.nmon(); jmon++ ){
    for( unsigned ivar=0; ivar<nvar(); ivar++ )
      iexp[ivar] = ( ivar<var.nvar()? var._expmon(jmon)[ivar]: 0 );
    var._coefmon[jmon] = _coefmon[_loc_expmon(iexp)];
    // Reset monomial coefficients to 0
    if( reset ) _coefmon[_loc_expmon(iexp)] = 0.;
  }
  delete[] iexp;
  *var._bndrem = 0.;
  var._unset_bndT();
  var._unset_bndpol();
  var._bndord_uptd = false;

  if( reset ){
    _unset_bndT();
    _unset_bndpol();
    _bndord_uptd = false;
  }
  return *this;
}

template <typename T> inline void
PolyVar<T>::_unset_bndpol
()
{
  delete _bndpol;
  _bndpol = 0;
}

template <typename T> inline void
PolyVar<T>::_set_bndpol
( const T*bndpol )
{
  if( !bndpol ){
    if( _bndpol ) delete _bndpol;
    _bndpol = 0;
  }
  else if( !_bndpol )
    _bndpol = new T( *bndpol );
  else
    *_bndpol = *bndpol;
}

template <typename T> inline void
PolyVar<T>::_set_bndpol
( const T&bndpol )
{
  if( !_bndpol )
    _bndpol = new T( bndpol );
  else
    *_bndpol = bndpol;
}

template <typename T> inline void
PolyVar<T>::_unset_bndT
()
{
  delete _bndT;
  _bndT = 0;
}

template <typename T> inline void
PolyVar<T>::_set_bndT
( const T*bndT )
{
  if( !bndT ){
    if( _bndT ) delete _bndT;
    _bndT = 0;
  }
  else if( !_bndT )
    _bndT = new T( *bndT );
  else
    *_bndT = *bndT;
}

template <typename T> inline void
PolyVar<T>::_set_bndT
( const T&bndT )
{
  if( !_bndT )
    _bndT = new T( bndT );
  else
    *_bndT = bndT;
}

template <typename T> inline void
PolyVar<T>::_center()
{
  const double remmid = Op<T>::mid(*_bndrem);
  _coefmon[0] += remmid;
  if( !_ndxmon.empty() ) _ndxmon.insert(0);
  if( _PM && _bndord_uptd ) _bndord[0] = _coefmon[0];
  *_bndrem -= remmid;
  if( _bndpol ) *_bndpol += remmid;
}

template <typename T> inline double
PolyVar<T>::coefmon
( const unsigned*iexp )
const
{
  if( !_PM ) return _coefmon[0];
  const unsigned imon = _PM->loc_expmon( iexp );
  return( imon<nmon()? _coefmon[imon]: 0. );
}

template <typename T> inline std::pair<unsigned, const double*>
PolyVar<T>::coefmon()
const
{
  return std::make_pair( (_PM?nmon():1), _coefmon );
}

template <typename T> inline std::pair<unsigned, const unsigned*>
PolyVar<T>::expmon()
const
{
  return std::make_pair( (_PM?nmon()*nvar():1), _PM->expmon() );
}

template <typename T> inline const unsigned*
PolyVar<T>::expmon
( const unsigned imon )
const
{
  return( _PM && imon<nmon()? _PM->expmon()+imon*nvar(): 0 );
}

template <typename T> inline const std::set<unsigned>&
PolyVar<T>::ndxmon()
const
{
  return _ndxmon;
}

} // namespace mc

#endif

