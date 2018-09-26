// Copyright (C) 2009-2016 Benoit Chachuat, Imperial College London.
// All Rights Reserved.
// This code is published under the Eclipse Public License.

#ifndef MC__SPOLYMODEL_H
#define MC__SPOLYMODEL_H

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

#undef  MC__SPOLYMODEL_DEBUG
#define MC__SPOLYMODEL_DEBUG_POLYBOUND
#define MC__SPOLYMODEL_CHECK

namespace mc
{
//! @brief C++ structure for ordering of monomial in graded lexicographic order (grlex)
template <class T> 
struct lt_expmon
{
  typedef std::pair< unsigned, std::map< unsigned, unsigned > > t_expmon;

  bool operator()
    ( const t_expmon&Exp1, const t_expmon&Exp2 ) const
    {
      // Order exponents based on their total order first
      if( Exp1.first < Exp2.first ) return true;
      if( Exp1.first > Exp2.first ) return false;
      // Account for the case of an empty list
      if( Exp1.second.empty() ) return true;
      if( Exp2.second.empty() ) return false;
      // Order in lexicographic order next
      for( auto it1=Exp1.second.begin(), it2=Exp2.second.begin(); it1!=Exp1.second.end(); ++it1, ++it2 ){
        if( it1->first < it2->first ) return true;
        if( it1->first > it2->first ) return false;
        if( it1->second > it2->second ) return true;
        if( it1->second < it2->second ) return false;
      }
      return false;
    }
};

//! @brief C++ base class for the computation of sparse polynomial models for factorable functions: Variable
////////////////////////////////////////////////////////////////////////
//! mc::SPolyVar is a C++ base class for definition of polynomial model
//! variables in sparse format.
////////////////////////////////////////////////////////////////////////
template <typename T>
class SPolyVar
////////////////////////////////////////////////////////////////////////
{
public:
  // Monomial representation: <total order, <variable index, order>>
  typedef std::pair< unsigned, std::map< unsigned, unsigned > > t_expmon;
  typedef std::map< t_expmon, double > t_coefmon;

protected:
  //! @brief Pointer to underlying sparse polynomial model environment
  //SPolyModel* _PM;

  //! @brief Array of size <tt>_nmon</tt> with monomial coefficients of variable
  t_coefmon _coefmon;

  //! @brief Remainder bound of variable
  T _bndrem;

  //! @brief Pointer to variable bound in underlying T arithmetic (possibly NULL if not computed)
  mutable T* _bndT;

  //! @brief Pointer to polynomial bound of variable (possibly NULL if not available)
  mutable T* _bndpol;

  //! @brief Initialize private/protected members of model variable
  void _init
    ();

  //! @brief Reinitialize private/protected members of model variable
  void _reinit
    ();

  //! @brief Clean up private/protected members of variable
  void _cleanup
    ();

  //! @brief Set variable equal to <a>var</a>
  SPolyVar<T>& _set
    ( const SPolyVar<T>&var );

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

public:
  /** @addtogroup POLYMOD Polynomial Model Arithmetic for Factorable Functions
   *  @{
   */
  //! @brief Constructor of variable linked to polynomial model environment <a>env</a>
  SPolyVar
    ()
    { _init(); }

  //! @brief Copy constructor of variable
  SPolyVar
    ( const SPolyVar<T>& var )
    { _init(); _set( var ); }

  //! @brief Destructor of variable
  virtual ~SPolyVar()
    { delete _bndpol; delete _bndT; }

  //! @brief Set multivariate polynomial coefficients in variable as <tt>coefmon</tt>
  virtual SPolyVar<T>& set
    ( t_coefmon& coefmon )
    { _coefmon = coefmon; _unset_bndT(); _unset_bndpol();
      return *this; } // this is assuming the same order and number of variables

  //! @brief Set remainder term in variable as <tt>bndrem</tt>
  virtual SPolyVar<T>& set
    ( const T& bndrem )
    { _bndrem = bndrem; return *this; }

  //! @brief Set multivariate polynomial coefficients and remainder term equal to those in variable <tt>var</tt>, possibly defined in another polynomial model environment with fewer variables or with a different expansion order. Coefficients involving other variables or higher order are initialized to 0 if <tt>reset=true</tt> (default), otherwise they are left unmodified. Higher-order terms in TV are bounded and added to the remainder bound.
  virtual SPolyVar<T>& set
    ( const SPolyVar<T>& var, const bool reset=true );
/*
  //! @brief Copy multivariate polynomial coefficients from current variable into variable <tt>var</tt>, possibly defined in another polynomial model environment with less variables or with a lower expansion order. Copied coefficients are reset to 0 in current Taylor variable if <tt>reset=true</tt>, otherwise they are left unmodified (default).
  virtual SPolyVar<T>& get
    ( SPolyVar<T>&var, const bool reset=false );
*/
  //! @brief Compute bound on variable using bounder <a>type</a>
  virtual T bound
    ( const int type ) const
    { if( !_bndT ) return _polybound(type) + _bndrem;
      else{ T bndT; return Op<T>::inter( bndT, _polybound(type) + _bndrem, *_bndT )?
                           bndT: _polybound(type) + _bndrem; } }

  //! @brief Retreive bound on variable using default bounder
  T bound() const
    { if( !_bndT ) return bndpol() + _bndrem;
      else{ T bndT; return Op<T>::inter( bndT, bndpol() + _bndrem, *_bndT )?
                           bndT: bndpol() + _bndrem; } }

  //! @brief Retreive bound on multivariate polynomial using default bounder
  T bndpol() const
    { if( !_bndpol ) _bndpol = new T( _polybound() );
      return *_bndpol; }

  //! @brief Retreive bound on all terms with (total) order <tt>minord</tt> or higher in polynomial part
  virtual T bndord
    ( const unsigned minord )
    const
    = 0;

  //! @brief Order of polynomial model
  virtual unsigned maxord
    ()
    const
    = 0;

  //! @brief Number of variables in polynomial model
  virtual unsigned nvar
    ()
    const
    = 0;

  //! @brief Total number of monomial terms in polynomial variable
  unsigned nmon
    ()
    const
    { return _coefmon.size(); };

  //! @brief Return remainder term of variable
  T remainder
    ()
    const
    { return _bndrem; }

  //! @brief Shortcut to mc::SPolyVar::bound
  virtual T B
    ( const int type )
    const
    { return bound( type ); }

  //! @brief Shortcut to mc::SPolyVar::bound
  virtual T B
    ()
    const
    { return bound(); }

  //! @brief Shortcut to mc::SPolyVar::remainder
  T R
    ()
    const
    { return remainder(); }

  //! @brief Get const map of monomial coefficients
  const t_coefmon& coefmon
    ()
    const
    { return _coefmon; }

  //! @brief Get map of monomial coefficients
  t_coefmon& coefmon
    ()
    { return _coefmon; }

  //! @brief Overloaded operator '=' for polynomial model variables
  virtual SPolyVar<T>& operator=
    ( const SPolyVar<T>& var )
    { _set( var ); return *this; }
  /** @} */
};

///////////////////////////////// SPolyVar /////////////////////////////////////

template <typename T> inline void
SPolyVar<T>::_init
()
{
  _bndpol = 0;
  _bndT = 0;
  _bndrem = 0.;
  return;
}

template <typename T> inline void
SPolyVar<T>::_cleanup
()
{
  delete _bndpol; _bndpol = 0;
  delete _bndT;   _bndT = 0;
  _coefmon.clear();
}

template <typename T> inline void
SPolyVar<T>::_reinit
()
{
  _cleanup();
  _init();
}

template <typename T> inline SPolyVar<T>&
SPolyVar<T>::_set
( const SPolyVar<T>&var )
{
  // Same SPolyVar?
  if( this == &var ) return *this;

  // Set coefficients and remainder
  //_reinit();
  _coefmon = var._coefmon;
  _bndrem  = var._bndrem;

  // Set polynomial bound
  _set_bndpol( var._bndpol );

  // Set underlying variable bound
  _set_bndT( var._bndT );

  return *this;
}

template <typename T> inline SPolyVar<T>&
SPolyVar<T>::set
( const SPolyVar<T>& var, const bool reset )
{
  if( reset ) _coefmon.clear();
  auto itord = var._coefmon.upper_bound( std::make_pair(maxord()+1,std::map<unsigned,unsigned>()) );
  _coefmon.insert( var._coefmon.begin(), itord );
  _bndrem  = var._bndrem + var.bndord(maxord()+1);  // var may have a higher order than *this
  _unset_bndT();
  _unset_bndpol();

  return *this;
}
/*
template <typename T> inline SPolyVar<T>&
SPolyVar<T>::get
( SPolyVar<T>& var, const bool reset )
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
*/
template <typename T> inline void
SPolyVar<T>::_unset_bndpol
()
{
  delete _bndpol;
  _bndpol = 0;
}

template <typename T> inline void
SPolyVar<T>::_set_bndpol
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
SPolyVar<T>::_set_bndpol
( const T&bndpol )
{
  if( !_bndpol )
    _bndpol = new T( bndpol );
  else
    *_bndpol = bndpol;
}

template <typename T> inline void
SPolyVar<T>::_unset_bndT
()
{
  delete _bndT;
  _bndT = 0;
}

template <typename T> inline void
SPolyVar<T>::_set_bndT
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
SPolyVar<T>::_set_bndT
( const T&bndT )
{
  if( !_bndT )
    _bndT = new T( bndT );
  else
    *_bndT = bndT;
}

template <typename T> inline void
SPolyVar<T>::_center()
{
  const double remmid = Op<T>::mid(_bndrem);
  if( remmid == 0. ) return;
  if( _coefmon.empty() || _coefmon.begin()->first.first ) 
    _coefmon.insert( std::make_pair( std::make_pair( 0, std::map<unsigned,unsigned>() ), remmid ) );
  else
    _coefmon.begin()->second += remmid;
  _bndrem -= remmid;
  if( _bndpol ) *_bndpol += remmid;
}

} // namespace mc

#endif

