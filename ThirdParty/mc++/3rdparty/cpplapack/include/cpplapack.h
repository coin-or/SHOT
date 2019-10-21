/*****************************************************************************/
/*                              cpplapack.h                                  */
/*****************************************************************************/

#ifndef CPPLAPACK_H
#define CPPLAPACK_H

//=============================================================================
#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <complex>
#include <cmath>
#include <cfloat>
#include <vector>//only for ?geev, ?gegv, etc.
#include <fstream>
#include <string>
#include <algorithm>
#include <stdint.h>

//=============================================================================
#ifdef  _MSC_VER
#ifdef  __INTEL_COMPILER
#include "prototype/mkl_lapack.h"
#else
#include "prototype/VC++.h"
#endif//__INTEL_COMPILER
#endif//_MSC_VER

//=============================================================================
#include "prototype/dblas.h"
#include "prototype/zblas.h"
#include "prototype/dlapack.h"
#include "prototype/zlapack.h"

//=============================================================================
#ifdef  __INTEL_COMPILER
//#include "prototype/sparse_dblas.h"
#endif//__INTEL_COMPILER

//=============================================================================
#ifdef  _OPENMP
#include <omp.h>
#endif//_OPENMP

//=============================================================================
/////// ERROR_REPORT ///////
#ifdef  __GNUC__
#define ERROR_REPORT std::cerr << "\n[ERROR]@" << __PRETTY_FUNCTION__ << " in \"" << __FILE__ << "\" at l." << __LINE__ << std::endl;
#else //__GNUC__
#define ERROR_REPORT std::cerr << "\n[ERROR]@" << __func__ << " in \"" << __FILE__ << "\" at l." << __LINE__ << std::endl;
#endif//__GNUC__

//=============================================================================
/////// WARNING_REPORT ///////
#ifdef  __GNUC__
#define WARNING_REPORT std::cerr << "\n[WARNING]@" << __PRETTY_FUNCTION__ << " in \"" << __FILE__ << "\" at l." << __LINE__ << std::endl;
#else //__GNUC__
#define WARNING_REPORT std::cerr << "\n[WARNING]@" << __func__ << " in \"" << __FILE__ << "\" at l." << __LINE__ << std::endl;
#endif//__GNUC__

//=============================================================================
/////// DEBUG_REPORT ///////
#ifdef  __GNUC__
#define DEBUG_REPORT std::cerr << "\n[DEBUG]@" << __PRETTY_FUNCTION__ << " in \"" << __FILE__ << "\" at l." << __LINE__ << std::endl;
#else //__GNUC__
#define DEBUG_REPORT std::cerr << "\n[DEBUG]@" << __func__ << " in \"" << __FILE__ << "\" at l." << __LINE__ << std::endl;
#endif//__GNUC__

//=============================================================================
/////// VERBOSE_REPORT ///////
#ifdef  CPPL_VERBOSE
#ifdef  __GNUC__
#define VERBOSE_REPORT std::cerr << "\n[VERBOSE]@" << __PRETTY_FUNCTION__ << " in \"" << __FILE__ << "\" at l." << __LINE__ << std::endl;
#else //__GNUC__
#define VERBOSE_REPORT std::cerr << "\n[VERBOSE]@" << __func__ << " in \"" << __FILE__ << "\" at l." << __LINE__ << std::endl;
#endif//__GNUC__
#else //CPPL_VERBOSE
#define VERBOSE_REPORT ;
#endif//CPPL_VERBOSE

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
namespace CPPL{ //!< namespace for CPPLapack

///////////////////////////////////////////////////////////
///////////////////// class prototype /////////////////////
///////////////////////////////////////////////////////////
class dcomponent;
class zcomponent;
class zhecomplex;
//////////////////////////////////
class dcovector; class _dcovector;
class drovector; class _drovector;
class dgematrix; class _dgematrix;
class dgbmatrix; class _dgbmatrix;
class dsymatrix; class _dsymatrix;
class dgsmatrix; class _dgsmatrix;
class dssmatrix; class _dssmatrix;
//////////////////////////////////
class zcovector; class _zcovector;
class zrovector; class _zrovector;
class zgematrix; class _zgematrix;
class zgbmatrix; class _zgbmatrix;
class zhematrix; class _zhematrix;
class zgsmatrix; class _zgsmatrix;
class zhsmatrix; class _zhsmatrix;
//////////////////////////////////
template<long>      class dcovector_small;
template<long>      class drovector_small;
template<long,long> class dgematrix_small;
template<long>      class dsymatrix_small;
//////////////////////////////////
template<long>      class zcovector_small;
template<long>      class zrovector_small;
template<long,long> class zgematrix_small;
template<long>      class zhematrix_small;

///////////////////////////////////////////////////////////
///////////////////////// typedef /////////////////////////
///////////////////////////////////////////////////////////
//////// complex ////////
typedef std::complex<double> comple;//double-precision complex
//////// double small ////////
typedef dcovector_small<2>   dcovec2;//2D column vector
typedef drovector_small<2>   drovec2;//2D row vector
typedef dgematrix_small<2,2> dgemat2;//2x2 dense matrix
typedef dsymatrix_small<2>   dsymat2;//2x2 symmetric matrix
typedef dcovector_small<3>   dcovec3;//3D column vector
typedef drovector_small<3>   drovec3;//3D row vector
typedef dcovector_small<4>   dquater;//quaternion(xi+yj+zk+r)
typedef dgematrix_small<3,3> dgemat3;//3x3 dense matrix
typedef dsymatrix_small<3>   dsymat3;//3x3 symmetric matrix
//////// complex small ////////
typedef zcovector_small<2>   zcovec2;//2D column vector
typedef zrovector_small<2>   zrovec2;//2D row vector
typedef zgematrix_small<2,2> zgemat2;//2x2 dense matrix
typedef zhematrix_small<2>   zhemat2;//2x2 hermitian matrix
typedef zcovector_small<3>   zcovec3;//3D column vector
typedef zrovector_small<3>   zrovec3;//3D row vector
typedef zgematrix_small<3,3> zgemat3;//3x3 dense matrix
typedef zhematrix_small<3>   zhemat3;//3x3 hermitian matrix

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////
//////////////////// class definition /////////////////////
///////////////////////////////////////////////////////////
#include "misc/dcomponent.hpp"
#include "misc/zcomponent.hpp"
#include "misc/zhecomplex.hpp"
///////////////////////////////////////
#include "dcovector-/dcovector.hpp"
#include "drovector-/drovector.hpp"
#include "dgematrix-/dgematrix.hpp"
#include "dgbmatrix-/dgbmatrix.hpp"
#include "dsymatrix-/dsymatrix.hpp"
#include "dgsmatrix-/dgsmatrix.hpp"
#include "dssmatrix-/dssmatrix.hpp"
///////////////////////////////////////
#include "_dcovector-/_dcovector.hpp"
#include "_drovector-/_drovector.hpp"
#include "_dgematrix-/_dgematrix.hpp"
#include "_dgbmatrix-/_dgbmatrix.hpp"
#include "_dsymatrix-/_dsymatrix.hpp"
#include "_dgsmatrix-/_dgsmatrix.hpp"
#include "_dssmatrix-/_dssmatrix.hpp"
///////////////////////////////////////
#include "zcovector-/zcovector.hpp"
#include "zrovector-/zrovector.hpp"
#include "zgematrix-/zgematrix.hpp"
#include "zgbmatrix-/zgbmatrix.hpp"
#include "zhematrix-/zhematrix.hpp"
#include "zgsmatrix-/zgsmatrix.hpp"
#include "zhsmatrix-/zhsmatrix.hpp"
///////////////////////////////////////
#include "_zcovector-/_zcovector.hpp"
#include "_zrovector-/_zrovector.hpp"
#include "_zgematrix-/_zgematrix.hpp"
#include "_zgbmatrix-/_zgbmatrix.hpp"
#include "_zhematrix-/_zhematrix.hpp"
#include "_zgsmatrix-/_zgsmatrix.hpp"
#include "_zhsmatrix-/_zhsmatrix.hpp"
///////////////////////////////////////
#include "small/dcovector_small.hpp"
#include "small/drovector_small.hpp"
#include "small/dgematrix_small.hpp"
#include "small/dsymatrix_small.hpp"
///////////////////////////////////////
#include "small/zcovector_small.hpp"
#include "small/zrovector_small.hpp"
#include "small/zgematrix_small.hpp"
#include "small/zhematrix_small.hpp"

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////
//////////////////////// dcovector ////////////////////////
///////////////////////////////////////////////////////////
#include "dcovector-/dcovector-constructor.hpp"
#include "dcovector-/dcovector-cast.hpp"
#include "dcovector-/dcovector-io.hpp"
#include "dcovector-/dcovector-calc.hpp"
#include "dcovector-/dcovector-misc.hpp"
#include "dcovector-/dcovector-unary.hpp"
#include "dcovector-/dcovector-dcovector.hpp"
#include "dcovector-/dcovector-_dcovector.hpp"
#include "dcovector-/dcovector-drovector.hpp"
#include "dcovector-/dcovector-_drovector.hpp"
#include "dcovector-/dcovector-dgematrix.hpp"
#include "dcovector-/dcovector-_dgematrix.hpp"
#include "dcovector-/dcovector-dgbmatrix.hpp"
#include "dcovector-/dcovector-_dgbmatrix.hpp"
#include "dcovector-/dcovector-dsymatrix.hpp"
#include "dcovector-/dcovector-_dsymatrix.hpp"
#include "dcovector-/dcovector-dgsmatrix.hpp"
#include "dcovector-/dcovector-_dgsmatrix.hpp"
//#include "dcovector-/dcovector-dssmatrix.hpp"
//#include "dcovector-/dcovector-_dssmatrix.hpp"
#include "dcovector-/dcovector-double.hpp"
/////////////////////// _dcovector ////////////////////////
#include "_dcovector-/_dcovector-constructor.hpp"
#include "_dcovector-/_dcovector-cast.hpp"
#include "_dcovector-/_dcovector-io.hpp"
#include "_dcovector-/_dcovector-calc.hpp"
#include "_dcovector-/_dcovector-misc.hpp"
#include "_dcovector-/_dcovector-unary.hpp"
#include "_dcovector-/_dcovector-dcovector.hpp"
#include "_dcovector-/_dcovector-_dcovector.hpp"
#include "_dcovector-/_dcovector-drovector.hpp"
#include "_dcovector-/_dcovector-_drovector.hpp"
#include "_dcovector-/_dcovector-dgematrix.hpp"
#include "_dcovector-/_dcovector-_dgematrix.hpp"
#include "_dcovector-/_dcovector-dgbmatrix.hpp"
#include "_dcovector-/_dcovector-_dgbmatrix.hpp"
#include "_dcovector-/_dcovector-dsymatrix.hpp"
#include "_dcovector-/_dcovector-_dsymatrix.hpp"
#include "_dcovector-/_dcovector-dgsmatrix.hpp"
#include "_dcovector-/_dcovector-_dgsmatrix.hpp"
//#include "_dcovector-/_dcovector-dssmatrix.hpp"
//#include "_dcovector-/_dcovector-_dssmatrix.hpp"
#include "_dcovector-/_dcovector-double.hpp"

///////////////////////////////////////////////////////////
//////////////////////// drovector ////////////////////////
///////////////////////////////////////////////////////////
#include "drovector-/drovector-constructor.hpp"
#include "drovector-/drovector-cast.hpp"
#include "drovector-/drovector-io.hpp"
#include "drovector-/drovector-calc.hpp"
#include "drovector-/drovector-misc.hpp"
#include "drovector-/drovector-unary.hpp"
#include "drovector-/drovector-dcovector.hpp"
#include "drovector-/drovector-_dcovector.hpp"
#include "drovector-/drovector-drovector.hpp"
#include "drovector-/drovector-_drovector.hpp"
#include "drovector-/drovector-dgematrix.hpp"
#include "drovector-/drovector-_dgematrix.hpp"
#include "drovector-/drovector-dgbmatrix.hpp"
#include "drovector-/drovector-_dgbmatrix.hpp"
#include "drovector-/drovector-dsymatrix.hpp"
#include "drovector-/drovector-_dsymatrix.hpp"
#include "drovector-/drovector-dgsmatrix.hpp"
#include "drovector-/drovector-_dgsmatrix.hpp"
#include "drovector-/drovector-dssmatrix.hpp"
#include "drovector-/drovector-_dssmatrix.hpp"
#include "drovector-/drovector-double.hpp"
/////////////////////// _drovector ////////////////////////
#include "_drovector-/_drovector-constructor.hpp"
#include "_drovector-/_drovector-cast.hpp"
#include "_drovector-/_drovector-io.hpp"
#include "_drovector-/_drovector-calc.hpp"
#include "_drovector-/_drovector-misc.hpp"
#include "_drovector-/_drovector-unary.hpp"
#include "_drovector-/_drovector-dcovector.hpp"
#include "_drovector-/_drovector-_dcovector.hpp"
#include "_drovector-/_drovector-drovector.hpp"
#include "_drovector-/_drovector-_drovector.hpp"
#include "_drovector-/_drovector-dgematrix.hpp"
#include "_drovector-/_drovector-_dgematrix.hpp"
#include "_drovector-/_drovector-dgbmatrix.hpp"
#include "_drovector-/_drovector-_dgbmatrix.hpp"
#include "_drovector-/_drovector-dsymatrix.hpp"
#include "_drovector-/_drovector-_dsymatrix.hpp"
#include "_drovector-/_drovector-dgsmatrix.hpp"
#include "_drovector-/_drovector-_dgsmatrix.hpp"
#include "_drovector-/_drovector-dssmatrix.hpp"
#include "_drovector-/_drovector-_dssmatrix.hpp"
#include "_drovector-/_drovector-double.hpp"

///////////////////////////////////////////////////////////
//////////////////////// dgematrix ////////////////////////
///////////////////////////////////////////////////////////
#include "dgematrix-/dgematrix-constructor.hpp"
#include "dgematrix-/dgematrix-cast.hpp"
#include "dgematrix-/dgematrix-io.hpp"
#include "dgematrix-/dgematrix-misc.hpp"
#include "dgematrix-/dgematrix-calc.hpp"
#include "dgematrix-/dgematrix-lapack.hpp"
#include "dgematrix-/dgematrix-unary.hpp"
#include "dgematrix-/dgematrix-dcovector.hpp"
#include "dgematrix-/dgematrix-_dcovector.hpp"
#include "dgematrix-/dgematrix-drovector.hpp"
#include "dgematrix-/dgematrix-_drovector.hpp"
#include "dgematrix-/dgematrix-dgematrix.hpp"
#include "dgematrix-/dgematrix-_dgematrix.hpp"
#include "dgematrix-/dgematrix-dgbmatrix.hpp"
#include "dgematrix-/dgematrix-_dgbmatrix.hpp"
#include "dgematrix-/dgematrix-dsymatrix.hpp"
#include "dgematrix-/dgematrix-_dsymatrix.hpp"
#include "dgematrix-/dgematrix-dgsmatrix.hpp"
#include "dgematrix-/dgematrix-_dgsmatrix.hpp"
//#include "dgematrix-/dgematrix-dssmatrix.hpp"
//#include "dgematrix-/dgematrix-_dssmatrix.hpp"
#include "dgematrix-/dgematrix-double.hpp"
/////////////////////// _dgematrix ////////////////////////
#include "_dgematrix-/_dgematrix-constructor.hpp"
#include "_dgematrix-/_dgematrix-cast.hpp"
#include "_dgematrix-/_dgematrix-io.hpp"
#include "_dgematrix-/_dgematrix-misc.hpp"
#include "_dgematrix-/_dgematrix-calc.hpp"
#include "_dgematrix-/_dgematrix-unary.hpp"
#include "_dgematrix-/_dgematrix-dcovector.hpp"
#include "_dgematrix-/_dgematrix-_dcovector.hpp"
#include "_dgematrix-/_dgematrix-drovector.hpp"
#include "_dgematrix-/_dgematrix-_drovector.hpp"
#include "_dgematrix-/_dgematrix-dgematrix.hpp"
#include "_dgematrix-/_dgematrix-_dgematrix.hpp"
#include "_dgematrix-/_dgematrix-dgbmatrix.hpp"
#include "_dgematrix-/_dgematrix-_dgbmatrix.hpp"
#include "_dgematrix-/_dgematrix-dsymatrix.hpp"
#include "_dgematrix-/_dgematrix-_dsymatrix.hpp"
#include "_dgematrix-/_dgematrix-dgsmatrix.hpp"
#include "_dgematrix-/_dgematrix-_dgsmatrix.hpp"
#include "_dgematrix-/_dgematrix-dssmatrix.hpp"
//#include "_dgematrix-/_dgematrix-_dssmatrix.hpp"
#include "_dgematrix-/_dgematrix-double.hpp"


///////////////////////////////////////////////////////////
//////////////////////// dgbmatrix ////////////////////////
///////////////////////////////////////////////////////////
#include "dgbmatrix-/dgbmatrix-constructor.hpp"
#include "dgbmatrix-/dgbmatrix-cast.hpp"
#include "dgbmatrix-/dgbmatrix-io.hpp"
#include "dgbmatrix-/dgbmatrix-misc.hpp"
#include "dgbmatrix-/dgbmatrix-calc.hpp"
#include "dgbmatrix-/dgbmatrix-lapack.hpp"
#include "dgbmatrix-/dgbmatrix-unary.hpp"
#include "dgbmatrix-/dgbmatrix-dcovector.hpp"
#include "dgbmatrix-/dgbmatrix-_dcovector.hpp"
#include "dgbmatrix-/dgbmatrix-drovector.hpp"
#include "dgbmatrix-/dgbmatrix-_drovector.hpp"
#include "dgbmatrix-/dgbmatrix-dgematrix.hpp"
#include "dgbmatrix-/dgbmatrix-_dgematrix.hpp"
#include "dgbmatrix-/dgbmatrix-dgbmatrix.hpp"
#include "dgbmatrix-/dgbmatrix-_dgbmatrix.hpp"
#include "dgbmatrix-/dgbmatrix-dsymatrix.hpp"
#include "dgbmatrix-/dgbmatrix-_dsymatrix.hpp"
#include "dgbmatrix-/dgbmatrix-dgsmatrix.hpp"
#include "dgbmatrix-/dgbmatrix-_dgsmatrix.hpp"
//#include "dgbmatrix-/dgbmatrix-dssmatrix.hpp"
//#include "dgbmatrix-/dgbmatrix-_dssmatrix.hpp"
#include "dgbmatrix-/dgbmatrix-double.hpp"
/////////////////////// _dgbmatrix ////////////////////////
#include "_dgbmatrix-/_dgbmatrix-constructor.hpp"
#include "_dgbmatrix-/_dgbmatrix-cast.hpp"
#include "_dgbmatrix-/_dgbmatrix-io.hpp"
#include "_dgbmatrix-/_dgbmatrix-misc.hpp"
#include "_dgbmatrix-/_dgbmatrix-calc.hpp"
#include "_dgbmatrix-/_dgbmatrix-unary.hpp"
#include "_dgbmatrix-/_dgbmatrix-dcovector.hpp"
#include "_dgbmatrix-/_dgbmatrix-_dcovector.hpp"
#include "_dgbmatrix-/_dgbmatrix-drovector.hpp"
#include "_dgbmatrix-/_dgbmatrix-_drovector.hpp"
#include "_dgbmatrix-/_dgbmatrix-dgematrix.hpp"
#include "_dgbmatrix-/_dgbmatrix-_dgematrix.hpp"
#include "_dgbmatrix-/_dgbmatrix-dgbmatrix.hpp"
#include "_dgbmatrix-/_dgbmatrix-_dgbmatrix.hpp"
#include "_dgbmatrix-/_dgbmatrix-dsymatrix.hpp"
#include "_dgbmatrix-/_dgbmatrix-_dsymatrix.hpp"
#include "_dgbmatrix-/_dgbmatrix-dgsmatrix.hpp"
#include "_dgbmatrix-/_dgbmatrix-_dgsmatrix.hpp"
//#include "_dgbmatrix-/_dgbmatrix-dssmatrix.hpp"
//#include "_dgbmatrix-/_dgbmatrix-_dssmatrix.hpp"
#include "_dgbmatrix-/_dgbmatrix-double.hpp"


///////////////////////////////////////////////////////////
//////////////////////// dsymatrix ////////////////////////
///////////////////////////////////////////////////////////
#include "dsymatrix-/dsymatrix-constructor.hpp"
#include "dsymatrix-/dsymatrix-cast.hpp"
#include "dsymatrix-/dsymatrix-io.hpp"
#include "dsymatrix-/dsymatrix-misc.hpp"
#include "dsymatrix-/dsymatrix-calc.hpp"
#include "dsymatrix-/dsymatrix-lapack.hpp"
#include "dsymatrix-/dsymatrix-unary.hpp"
#include "dsymatrix-/dsymatrix-dcovector.hpp"
#include "dsymatrix-/dsymatrix-_dcovector.hpp"
#include "dsymatrix-/dsymatrix-drovector.hpp"
#include "dsymatrix-/dsymatrix-_drovector.hpp"
#include "dsymatrix-/dsymatrix-dgematrix.hpp"
#include "dsymatrix-/dsymatrix-_dgematrix.hpp"
#include "dsymatrix-/dsymatrix-dgbmatrix.hpp"
#include "dsymatrix-/dsymatrix-_dgbmatrix.hpp"
#include "dsymatrix-/dsymatrix-dsymatrix.hpp"
#include "dsymatrix-/dsymatrix-_dsymatrix.hpp"
#include "dsymatrix-/dsymatrix-dgsmatrix.hpp"
#include "dsymatrix-/dsymatrix-_dgsmatrix.hpp"
//#include "dsymatrix-/dsymatrix-dssmatrix.hpp"
//#include "dsymatrix-/dsymatrix-_dssmatrix.hpp"
#include "dsymatrix-/dsymatrix-double.hpp"
/////////////////////// _dsymatrix ////////////////////////
#include "_dsymatrix-/_dsymatrix-constructor.hpp"
#include "_dsymatrix-/_dsymatrix-cast.hpp"
#include "_dsymatrix-/_dsymatrix-io.hpp"
#include "_dsymatrix-/_dsymatrix-misc.hpp"
#include "_dsymatrix-/_dsymatrix-calc.hpp"
#include "_dsymatrix-/_dsymatrix-unary.hpp"
#include "_dsymatrix-/_dsymatrix-dcovector.hpp"
#include "_dsymatrix-/_dsymatrix-_dcovector.hpp"
#include "_dsymatrix-/_dsymatrix-drovector.hpp"
#include "_dsymatrix-/_dsymatrix-_drovector.hpp"
#include "_dsymatrix-/_dsymatrix-dgematrix.hpp"
#include "_dsymatrix-/_dsymatrix-_dgematrix.hpp"
#include "_dsymatrix-/_dsymatrix-dgbmatrix.hpp"
#include "_dsymatrix-/_dsymatrix-_dgbmatrix.hpp"
#include "_dsymatrix-/_dsymatrix-dsymatrix.hpp"
#include "_dsymatrix-/_dsymatrix-_dsymatrix.hpp"
#include "_dsymatrix-/_dsymatrix-dgsmatrix.hpp"
#include "_dsymatrix-/_dsymatrix-_dgsmatrix.hpp"
//#include "_dsymatrix-/_dsymatrix-dssmatrix.hpp"
//#include "_dsymatrix-/_dsymatrix-_dssmatrix.hpp"
#include "_dsymatrix-/_dsymatrix-double.hpp"

///////////////////////////////////////////////////////////
//////////////////////// dgsmatrix ////////////////////////
///////////////////////////////////////////////////////////
#include "dgsmatrix-/dgsmatrix-constructor.hpp"
#include "dgsmatrix-/dgsmatrix-cast.hpp"
#include "dgsmatrix-/dgsmatrix-io.hpp"
#include "dgsmatrix-/dgsmatrix-misc.hpp"
#include "dgsmatrix-/dgsmatrix-calc.hpp"
#include "dgsmatrix-/dgsmatrix-unary.hpp"
#include "dgsmatrix-/dgsmatrix-dcovector.hpp"
#include "dgsmatrix-/dgsmatrix-_dcovector.hpp"
#include "dgsmatrix-/dgsmatrix-drovector.hpp"
#include "dgsmatrix-/dgsmatrix-_drovector.hpp"
#include "dgsmatrix-/dgsmatrix-dgematrix.hpp"
#include "dgsmatrix-/dgsmatrix-_dgematrix.hpp"
#include "dgsmatrix-/dgsmatrix-dgbmatrix.hpp"
#include "dgsmatrix-/dgsmatrix-_dgbmatrix.hpp"
#include "dgsmatrix-/dgsmatrix-dsymatrix.hpp"
#include "dgsmatrix-/dgsmatrix-_dsymatrix.hpp"
#include "dgsmatrix-/dgsmatrix-dgsmatrix.hpp"
#include "dgsmatrix-/dgsmatrix-_dgsmatrix.hpp"
//#include "dgsmatrix-/dgsmatrix-dssmatrix.hpp"
//#include "dgsmatrix-/dgsmatrix-_dssmatrix.hpp"
#include "dgsmatrix-/dgsmatrix-double.hpp"
/////////////////////// _dgsmatrix ////////////////////////
#include "_dgsmatrix-/_dgsmatrix-constructor.hpp"
#include "_dgsmatrix-/_dgsmatrix-cast.hpp"
#include "_dgsmatrix-/_dgsmatrix-io.hpp"
#include "_dgsmatrix-/_dgsmatrix-misc.hpp"
#include "_dgsmatrix-/_dgsmatrix-calc.hpp"
#include "_dgsmatrix-/_dgsmatrix-unary.hpp"
#include "_dgsmatrix-/_dgsmatrix-dcovector.hpp"
#include "_dgsmatrix-/_dgsmatrix-_dcovector.hpp"
#include "_dgsmatrix-/_dgsmatrix-drovector.hpp"
#include "_dgsmatrix-/_dgsmatrix-_drovector.hpp"
#include "_dgsmatrix-/_dgsmatrix-dgematrix.hpp"
#include "_dgsmatrix-/_dgsmatrix-_dgematrix.hpp"
#include "_dgsmatrix-/_dgsmatrix-dgbmatrix.hpp"
#include "_dgsmatrix-/_dgsmatrix-_dgbmatrix.hpp"
#include "_dgsmatrix-/_dgsmatrix-dsymatrix.hpp"
#include "_dgsmatrix-/_dgsmatrix-_dsymatrix.hpp"
#include "_dgsmatrix-/_dgsmatrix-dgsmatrix.hpp"
#include "_dgsmatrix-/_dgsmatrix-_dgsmatrix.hpp"
//#include "_dgsmatrix-/_dgsmatrix-dssmatrix.hpp"
//#include "_dgsmatrix-/_dgsmatrix-_dssmatrix.hpp"
#include "_dgsmatrix-/_dgsmatrix-double.hpp"

///////////////////////////////////////////////////////////
//////////////////////// dssmatrix ////////////////////////
///////////////////////////////////////////////////////////
#include "dssmatrix-/dssmatrix-constructor.hpp"
#include "dssmatrix-/dssmatrix-cast.hpp"
#include "dssmatrix-/dssmatrix-io.hpp"
#include "dssmatrix-/dssmatrix-misc.hpp"
#include "dssmatrix-/dssmatrix-calc.hpp"
#include "dssmatrix-/dssmatrix-unary.hpp"
#include "dssmatrix-/dssmatrix-dcovector.hpp"
#include "dssmatrix-/dssmatrix-_dcovector.hpp"
#include "dssmatrix-/dssmatrix-drovector.hpp"
#include "dssmatrix-/dssmatrix-_drovector.hpp"
#include "dssmatrix-/dssmatrix-dgematrix.hpp"
#include "dssmatrix-/dssmatrix-_dgematrix.hpp"
#include "dssmatrix-/dssmatrix-dgbmatrix.hpp"
#include "dssmatrix-/dssmatrix-_dgbmatrix.hpp"
#include "dssmatrix-/dssmatrix-dsymatrix.hpp"
#include "dssmatrix-/dssmatrix-_dsymatrix.hpp"
#include "dssmatrix-/dssmatrix-dssmatrix.hpp"
#include "dssmatrix-/dssmatrix-_dssmatrix.hpp"
#include "dssmatrix-/dssmatrix-double.hpp"
/////////////////////// _dssmatrix ////////////////////////
#include "_dssmatrix-/_dssmatrix-constructor.hpp"
#include "_dssmatrix-/_dssmatrix-cast.hpp"
#include "_dssmatrix-/_dssmatrix-io.hpp"
#include "_dssmatrix-/_dssmatrix-misc.hpp"
#include "_dssmatrix-/_dssmatrix-calc.hpp"
#include "_dssmatrix-/_dssmatrix-unary.hpp"
#include "_dssmatrix-/_dssmatrix-dcovector.hpp"
#include "_dssmatrix-/_dssmatrix-_dcovector.hpp"
#include "_dssmatrix-/_dssmatrix-drovector.hpp"
#include "_dssmatrix-/_dssmatrix-_drovector.hpp"
#include "_dssmatrix-/_dssmatrix-dgematrix.hpp"
#include "_dssmatrix-/_dssmatrix-_dgematrix.hpp"
#include "_dssmatrix-/_dssmatrix-dgbmatrix.hpp"
#include "_dssmatrix-/_dssmatrix-_dgbmatrix.hpp"
#include "_dssmatrix-/_dssmatrix-dsymatrix.hpp"
#include "_dssmatrix-/_dssmatrix-_dsymatrix.hpp"
#include "_dssmatrix-/_dssmatrix-dssmatrix.hpp"
#include "_dssmatrix-/_dssmatrix-_dssmatrix.hpp"
#include "_dssmatrix-/_dssmatrix-double.hpp"

///////////////////////////////////////////////////////////
///////////////////////// double //////////////////////////
///////////////////////////////////////////////////////////
#include "double-/double-dcovector.hpp"
#include "double-/double-_dcovector.hpp"
#include "double-/double-dgematrix.hpp"
#include "double-/double-_dgematrix.hpp"
#include "double-/double-dgbmatrix.hpp"
#include "double-/double-_dgbmatrix.hpp"
#include "double-/double-dsymatrix.hpp"
#include "double-/double-_dsymatrix.hpp"
#include "double-/double-dgsmatrix.hpp"
#include "double-/double-_dgsmatrix.hpp"
#include "double-/double-dssmatrix.hpp"
#include "double-/double-_dssmatrix.hpp"
#include "double-/double-drovector.hpp"
#include "double-/double-_drovector.hpp"

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////
//////////////////////// zrovector ////////////////////////
///////////////////////////////////////////////////////////
#include "zrovector-/zrovector-constructor.hpp"
#include "zrovector-/zrovector-io.hpp"
#include "zrovector-/zrovector-calc.hpp"
#include "zrovector-/zrovector-misc.hpp"
#include "zrovector-/zrovector-unary.hpp"
#include "zrovector-/zrovector-zcovector.hpp"
#include "zrovector-/zrovector-_zcovector.hpp"
#include "zrovector-/zrovector-zrovector.hpp"
#include "zrovector-/zrovector-_zrovector.hpp"
#include "zrovector-/zrovector-zgematrix.hpp"
#include "zrovector-/zrovector-_zgematrix.hpp"
#include "zrovector-/zrovector-zgbmatrix.hpp"
#include "zrovector-/zrovector-_zgbmatrix.hpp"
#include "zrovector-/zrovector-zhematrix.hpp"
#include "zrovector-/zrovector-_zhematrix.hpp"
#include "zrovector-/zrovector-zgsmatrix.hpp"
#include "zrovector-/zrovector-_zgsmatrix.hpp"
#include "zrovector-/zrovector-zhsmatrix.hpp"
#include "zrovector-/zrovector-_zhsmatrix.hpp"
#include "zrovector-/zrovector-double.hpp"
#include "zrovector-/zrovector-complex.hpp"
/////////////////////// _zrovector ////////////////////////
#include "_zrovector-/_zrovector-constructor.hpp"
#include "_zrovector-/_zrovector-io.hpp"
#include "_zrovector-/_zrovector-calc.hpp"
#include "_zrovector-/_zrovector-misc.hpp"
#include "_zrovector-/_zrovector-unary.hpp"
#include "_zrovector-/_zrovector-zcovector.hpp"
#include "_zrovector-/_zrovector-_zcovector.hpp"
#include "_zrovector-/_zrovector-zrovector.hpp"
#include "_zrovector-/_zrovector-_zrovector.hpp"
#include "_zrovector-/_zrovector-zgematrix.hpp"
#include "_zrovector-/_zrovector-_zgematrix.hpp"
#include "_zrovector-/_zrovector-zgbmatrix.hpp"
#include "_zrovector-/_zrovector-_zgbmatrix.hpp"
#include "_zrovector-/_zrovector-zhematrix.hpp"
#include "_zrovector-/_zrovector-_zhematrix.hpp"
#include "_zrovector-/_zrovector-zgsmatrix.hpp"
#include "_zrovector-/_zrovector-_zgsmatrix.hpp"
#include "_zrovector-/_zrovector-zhsmatrix.hpp"
#include "_zrovector-/_zrovector-_zhsmatrix.hpp"
#include "_zrovector-/_zrovector-double.hpp"
#include "_zrovector-/_zrovector-complex.hpp"

///////////////////////////////////////////////////////////
//////////////////////// zgematrix ////////////////////////
///////////////////////////////////////////////////////////
#include "zgematrix-/zgematrix-constructor.hpp"
#include "zgematrix-/zgematrix-cast.hpp"
#include "zgematrix-/zgematrix-io.hpp"
#include "zgematrix-/zgematrix-misc.hpp"
#include "zgematrix-/zgematrix-calc.hpp"
#include "zgematrix-/zgematrix-lapack.hpp"
#include "zgematrix-/zgematrix-unary.hpp"
#include "zgematrix-/zgematrix-zcovector.hpp"
#include "zgematrix-/zgematrix-_zcovector.hpp"
#include "zgematrix-/zgematrix-zrovector.hpp"
#include "zgematrix-/zgematrix-_zrovector.hpp"
#include "zgematrix-/zgematrix-zgematrix.hpp"
#include "zgematrix-/zgematrix-_zgematrix.hpp"
#include "zgematrix-/zgematrix-zgbmatrix.hpp"
#include "zgematrix-/zgematrix-_zgbmatrix.hpp"
#include "zgematrix-/zgematrix-zhematrix.hpp"
#include "zgematrix-/zgematrix-_zhematrix.hpp"
#include "zgematrix-/zgematrix-zgsmatrix.hpp"
#include "zgematrix-/zgematrix-_zgsmatrix.hpp"
//#include "zgematrix-/zgematrix-zhsmatrix.hpp"
//#include "zgematrix-/zgematrix-_zhsmatrix.hpp"
#include "zgematrix-/zgematrix-double.hpp"
#include "zgematrix-/zgematrix-complex.hpp"
/////////////////////// _zgematrix ////////////////////////
#include "_zgematrix-/_zgematrix-constructor.hpp"
#include "_zgematrix-/_zgematrix-cast.hpp"
#include "_zgematrix-/_zgematrix-io.hpp"
#include "_zgematrix-/_zgematrix-misc.hpp"
#include "_zgematrix-/_zgematrix-calc.hpp"
#include "_zgematrix-/_zgematrix-unary.hpp"
#include "_zgematrix-/_zgematrix-zcovector.hpp"
#include "_zgematrix-/_zgematrix-_zcovector.hpp"
#include "_zgematrix-/_zgematrix-zrovector.hpp"
#include "_zgematrix-/_zgematrix-_zrovector.hpp"
#include "_zgematrix-/_zgematrix-zgematrix.hpp"
#include "_zgematrix-/_zgematrix-_zgematrix.hpp"
#include "_zgematrix-/_zgematrix-zgbmatrix.hpp"
#include "_zgematrix-/_zgematrix-_zgbmatrix.hpp"
#include "_zgematrix-/_zgematrix-zhematrix.hpp"
#include "_zgematrix-/_zgematrix-_zhematrix.hpp"
#include "_zgematrix-/_zgematrix-zgsmatrix.hpp"
#include "_zgematrix-/_zgematrix-_zgsmatrix.hpp"
//#include "_zgematrix-/_zgematrix-zhsmatrix.hpp"
//#include "_zgematrix-/_zgematrix-_zhsmatrix.hpp"
#include "_zgematrix-/_zgematrix-double.hpp"
#include "_zgematrix-/_zgematrix-complex.hpp"


///////////////////////////////////////////////////////////
//////////////////////// zgbmatrix ////////////////////////
///////////////////////////////////////////////////////////
#include "zgbmatrix-/zgbmatrix-constructor.hpp"
#include "zgbmatrix-/zgbmatrix-cast.hpp"
#include "zgbmatrix-/zgbmatrix-io.hpp"
#include "zgbmatrix-/zgbmatrix-misc.hpp"
#include "zgbmatrix-/zgbmatrix-calc.hpp"
#include "zgbmatrix-/zgbmatrix-lapack.hpp"
#include "zgbmatrix-/zgbmatrix-unary.hpp"
#include "zgbmatrix-/zgbmatrix-zcovector.hpp"
#include "zgbmatrix-/zgbmatrix-_zcovector.hpp"
#include "zgbmatrix-/zgbmatrix-zrovector.hpp"
#include "zgbmatrix-/zgbmatrix-_zrovector.hpp"
#include "zgbmatrix-/zgbmatrix-zgematrix.hpp"
#include "zgbmatrix-/zgbmatrix-_zgematrix.hpp"
#include "zgbmatrix-/zgbmatrix-zgbmatrix.hpp"
#include "zgbmatrix-/zgbmatrix-_zgbmatrix.hpp"
#include "zgbmatrix-/zgbmatrix-zhematrix.hpp"
#include "zgbmatrix-/zgbmatrix-_zhematrix.hpp"
//#include "zgbmatrix-/zgbmatrix-zgsmatrix.hpp"
//#include "zgbmatrix-/zgbmatrix-_zgsmatrix.hpp"
//#include "zgbmatrix-/zgbmatrix-zhsmatrix.hpp"
//#include "zgbmatrix-/zgbmatrix-_zhsmatrix.hpp"
#include "zgbmatrix-/zgbmatrix-double.hpp"
#include "zgbmatrix-/zgbmatrix-complex.hpp"
/////////////////////// _zgbmatrix ////////////////////////
#include "_zgbmatrix-/_zgbmatrix-constructor.hpp"
#include "_zgbmatrix-/_zgbmatrix-cast.hpp"
#include "_zgbmatrix-/_zgbmatrix-io.hpp"
#include "_zgbmatrix-/_zgbmatrix-misc.hpp"
#include "_zgbmatrix-/_zgbmatrix-calc.hpp"
#include "_zgbmatrix-/_zgbmatrix-unary.hpp"
#include "_zgbmatrix-/_zgbmatrix-zcovector.hpp"
#include "_zgbmatrix-/_zgbmatrix-_zcovector.hpp"
#include "_zgbmatrix-/_zgbmatrix-zrovector.hpp"
#include "_zgbmatrix-/_zgbmatrix-_zrovector.hpp"
#include "_zgbmatrix-/_zgbmatrix-zgematrix.hpp"
#include "_zgbmatrix-/_zgbmatrix-_zgematrix.hpp"
#include "_zgbmatrix-/_zgbmatrix-zgbmatrix.hpp"
#include "_zgbmatrix-/_zgbmatrix-_zgbmatrix.hpp"
#include "_zgbmatrix-/_zgbmatrix-zhematrix.hpp"
#include "_zgbmatrix-/_zgbmatrix-_zhematrix.hpp"
//#include "_zgbmatrix-/_zgbmatrix-zgsmatrix.hpp"
//#include "_zgbmatrix-/_zgbmatrix-_zgsmatrix.hpp"
//#include "_zgbmatrix-/_zgbmatrix-zhsmatrix.hpp"
//#include "_zgbmatrix-/_zgbmatrix-_zhsmatrix.hpp"
#include "_zgbmatrix-/_zgbmatrix-double.hpp"
#include "_zgbmatrix-/_zgbmatrix-complex.hpp"


///////////////////////////////////////////////////////////
//////////////////////// zhematrix ////////////////////////
///////////////////////////////////////////////////////////
#include "zhematrix-/zhematrix-constructor.hpp"
#include "zhematrix-/zhematrix-cast.hpp"
#include "zhematrix-/zhematrix-io.hpp"
#include "zhematrix-/zhematrix-misc.hpp"
#include "zhematrix-/zhematrix-calc.hpp"
#include "zhematrix-/zhematrix-lapack.hpp"
#include "zhematrix-/zhematrix-unary.hpp"
#include "zhematrix-/zhematrix-zcovector.hpp"
#include "zhematrix-/zhematrix-_zcovector.hpp"
#include "zhematrix-/zhematrix-zrovector.hpp"
#include "zhematrix-/zhematrix-_zrovector.hpp"
#include "zhematrix-/zhematrix-zgematrix.hpp"
#include "zhematrix-/zhematrix-_zgematrix.hpp"
#include "zhematrix-/zhematrix-zgbmatrix.hpp"
#include "zhematrix-/zhematrix-_zgbmatrix.hpp"
#include "zhematrix-/zhematrix-zhematrix.hpp"
#include "zhematrix-/zhematrix-_zhematrix.hpp"
//#include "zhematrix-/zhematrix-zhsmatrix.hpp"
//#include "zhematrix-/zhematrix-_zhsmatrix.hpp"
#include "zhematrix-/zhematrix-double.hpp"
#include "zhematrix-/zhematrix-complex.hpp"
/////////////////////// _zhematrix ////////////////////////
#include "_zhematrix-/_zhematrix-constructor.hpp"
#include "_zhematrix-/_zhematrix-cast.hpp"
#include "_zhematrix-/_zhematrix-io.hpp"
#include "_zhematrix-/_zhematrix-misc.hpp"
#include "_zhematrix-/_zhematrix-calc.hpp"
#include "_zhematrix-/_zhematrix-unary.hpp"
#include "_zhematrix-/_zhematrix-zcovector.hpp"
#include "_zhematrix-/_zhematrix-_zcovector.hpp"
#include "_zhematrix-/_zhematrix-zrovector.hpp"
#include "_zhematrix-/_zhematrix-_zrovector.hpp"
#include "_zhematrix-/_zhematrix-zgematrix.hpp"
#include "_zhematrix-/_zhematrix-_zgematrix.hpp"
#include "_zhematrix-/_zhematrix-zgbmatrix.hpp"
#include "_zhematrix-/_zhematrix-_zgbmatrix.hpp"
#include "_zhematrix-/_zhematrix-zhematrix.hpp"
#include "_zhematrix-/_zhematrix-_zhematrix.hpp"
//#include "_zhematrix-/_zhematrix-zhsmatrix.hpp"
//#include "_zhematrix-/_zhematrix-_zhsmatrix.hpp"
#include "_zhematrix-/_zhematrix-double.hpp"
#include "_zhematrix-/_zhematrix-complex.hpp"


///////////////////////////////////////////////////////////
//////////////////////// zgsmatrix ////////////////////////
///////////////////////////////////////////////////////////
#include "zgsmatrix-/zgsmatrix-constructor.hpp"
#include "zgsmatrix-/zgsmatrix-cast.hpp"
#include "zgsmatrix-/zgsmatrix-io.hpp"
#include "zgsmatrix-/zgsmatrix-misc.hpp"
#include "zgsmatrix-/zgsmatrix-calc.hpp"
#include "zgsmatrix-/zgsmatrix-unary.hpp"
#include "zgsmatrix-/zgsmatrix-zcovector.hpp"
#include "zgsmatrix-/zgsmatrix-_zcovector.hpp"
#include "zgsmatrix-/zgsmatrix-zrovector.hpp"
#include "zgsmatrix-/zgsmatrix-_zrovector.hpp"
#include "zgsmatrix-/zgsmatrix-zgematrix.hpp"
#include "zgsmatrix-/zgsmatrix-_zgematrix.hpp"
#include "zgsmatrix-/zgsmatrix-zgbmatrix.hpp"
#include "zgsmatrix-/zgsmatrix-_zgbmatrix.hpp"
#include "zgsmatrix-/zgsmatrix-zhematrix.hpp"
#include "zgsmatrix-/zgsmatrix-_zhematrix.hpp"
#include "zgsmatrix-/zgsmatrix-zgsmatrix.hpp"
#include "zgsmatrix-/zgsmatrix-_zgsmatrix.hpp"
//#include "zgsmatrix-/zgsmatrix-zhsmatrix.hpp"
//#include "zgsmatrix-/zgsmatrix-_zhsmatrix.hpp"
#include "zgsmatrix-/zgsmatrix-double.hpp"
#include "zgsmatrix-/zgsmatrix-complex.hpp"
/////////////////////// _zgsmatrix ////////////////////////
#include "_zgsmatrix-/_zgsmatrix-constructor.hpp"
#include "_zgsmatrix-/_zgsmatrix-cast.hpp"
#include "_zgsmatrix-/_zgsmatrix-io.hpp"
#include "_zgsmatrix-/_zgsmatrix-misc.hpp"
#include "_zgsmatrix-/_zgsmatrix-calc.hpp"
#include "_zgsmatrix-/_zgsmatrix-unary.hpp"
#include "_zgsmatrix-/_zgsmatrix-zcovector.hpp"
#include "_zgsmatrix-/_zgsmatrix-_zcovector.hpp"
#include "_zgsmatrix-/_zgsmatrix-zrovector.hpp"
#include "_zgsmatrix-/_zgsmatrix-_zrovector.hpp"
#include "_zgsmatrix-/_zgsmatrix-zgematrix.hpp"
#include "_zgsmatrix-/_zgsmatrix-_zgematrix.hpp"
#include "_zgsmatrix-/_zgsmatrix-zgbmatrix.hpp"
#include "_zgsmatrix-/_zgsmatrix-_zgbmatrix.hpp"
#include "_zgsmatrix-/_zgsmatrix-zhematrix.hpp"
#include "_zgsmatrix-/_zgsmatrix-_zhematrix.hpp"
#include "_zgsmatrix-/_zgsmatrix-zgsmatrix.hpp"
#include "_zgsmatrix-/_zgsmatrix-_zgsmatrix.hpp"
//#include "_zgsmatrix-/_zgsmatrix-zhsmatrix.hpp"
//#include "_zgsmatrix-/_zgsmatrix-_zhsmatrix.hpp"
#include "_zgsmatrix-/_zgsmatrix-double.hpp"
#include "_zgsmatrix-/_zgsmatrix-complex.hpp"

///////////////////////////////////////////////////////////
//////////////////////// zhsmatrix ////////////////////////
///////////////////////////////////////////////////////////
#include "zhsmatrix-/zhsmatrix-constructor.hpp"
#include "zhsmatrix-/zhsmatrix-cast.hpp"
#include "zhsmatrix-/zhsmatrix-io.hpp"
#include "zhsmatrix-/zhsmatrix-misc.hpp"
#include "zhsmatrix-/zhsmatrix-calc.hpp"
#include "zhsmatrix-/zhsmatrix-unary.hpp"
#include "zhsmatrix-/zhsmatrix-zcovector.hpp"
#include "zhsmatrix-/zhsmatrix-_zcovector.hpp"
#include "zhsmatrix-/zhsmatrix-zrovector.hpp"
#include "zhsmatrix-/zhsmatrix-_zrovector.hpp"
#include "zhsmatrix-/zhsmatrix-zgematrix.hpp"
#include "zhsmatrix-/zhsmatrix-_zgematrix.hpp"
#include "zhsmatrix-/zhsmatrix-zgbmatrix.hpp"
#include "zhsmatrix-/zhsmatrix-_zgbmatrix.hpp"
#include "zhsmatrix-/zhsmatrix-zhematrix.hpp"
#include "zhsmatrix-/zhsmatrix-_zhematrix.hpp"
#include "zhsmatrix-/zhsmatrix-zhsmatrix.hpp"
#include "zhsmatrix-/zhsmatrix-_zhsmatrix.hpp"
#include "zhsmatrix-/zhsmatrix-double.hpp"
#include "zhsmatrix-/zhsmatrix-complex.hpp"
/////////////////////// _zhsmatrix ////////////////////////
#include "_zhsmatrix-/_zhsmatrix-constructor.hpp"
#include "_zhsmatrix-/_zhsmatrix-cast.hpp"
#include "_zhsmatrix-/_zhsmatrix-io.hpp"
#include "_zhsmatrix-/_zhsmatrix-misc.hpp"
#include "_zhsmatrix-/_zhsmatrix-calc.hpp"
#include "_zhsmatrix-/_zhsmatrix-unary.hpp"
#include "_zhsmatrix-/_zhsmatrix-zcovector.hpp"
#include "_zhsmatrix-/_zhsmatrix-_zcovector.hpp"
#include "_zhsmatrix-/_zhsmatrix-zrovector.hpp"
#include "_zhsmatrix-/_zhsmatrix-_zrovector.hpp"
#include "_zhsmatrix-/_zhsmatrix-zgematrix.hpp"
#include "_zhsmatrix-/_zhsmatrix-_zgematrix.hpp"
#include "_zhsmatrix-/_zhsmatrix-zgbmatrix.hpp"
#include "_zhsmatrix-/_zhsmatrix-_zgbmatrix.hpp"
#include "_zhsmatrix-/_zhsmatrix-zhematrix.hpp"
#include "_zhsmatrix-/_zhsmatrix-_zhematrix.hpp"
#include "_zhsmatrix-/_zhsmatrix-zhsmatrix.hpp"
#include "_zhsmatrix-/_zhsmatrix-_zhsmatrix.hpp"
#include "_zhsmatrix-/_zhsmatrix-double.hpp"
#include "_zhsmatrix-/_zhsmatrix-complex.hpp"


///////////////////////////////////////////////////////////
//////////////////////// zcovector ////////////////////////
///////////////////////////////////////////////////////////
#include "zcovector-/zcovector-constructor.hpp"
#include "zcovector-/zcovector-io.hpp"
#include "zcovector-/zcovector-calc.hpp"
#include "zcovector-/zcovector-misc.hpp"
#include "zcovector-/zcovector-unary.hpp"
#include "zcovector-/zcovector-zcovector.hpp"
#include "zcovector-/zcovector-_zcovector.hpp"
#include "zcovector-/zcovector-zrovector.hpp"
#include "zcovector-/zcovector-_zrovector.hpp"
#include "zcovector-/zcovector-zgematrix.hpp"
#include "zcovector-/zcovector-_zgematrix.hpp"
#include "zcovector-/zcovector-zgbmatrix.hpp"
#include "zcovector-/zcovector-_zgbmatrix.hpp"
#include "zcovector-/zcovector-zhematrix.hpp"
#include "zcovector-/zcovector-_zhematrix.hpp"
#include "zcovector-/zcovector-zgsmatrix.hpp"
#include "zcovector-/zcovector-_zgsmatrix.hpp"
#include "zcovector-/zcovector-zhsmatrix.hpp"
#include "zcovector-/zcovector-_zhsmatrix.hpp"
#include "zcovector-/zcovector-double.hpp"
#include "zcovector-/zcovector-complex.hpp"
/////////////////////// _zcovector ////////////////////////
#include "_zcovector-/_zcovector-constructor.hpp"
#include "_zcovector-/_zcovector-io.hpp"
#include "_zcovector-/_zcovector-calc.hpp"
#include "_zcovector-/_zcovector-misc.hpp"
#include "_zcovector-/_zcovector-unary.hpp"
#include "_zcovector-/_zcovector-zcovector.hpp"
#include "_zcovector-/_zcovector-_zcovector.hpp"
#include "_zcovector-/_zcovector-zrovector.hpp"
#include "_zcovector-/_zcovector-_zrovector.hpp"
#include "_zcovector-/_zcovector-zgematrix.hpp"
#include "_zcovector-/_zcovector-_zgematrix.hpp"
#include "_zcovector-/_zcovector-zgbmatrix.hpp"
#include "_zcovector-/_zcovector-_zgbmatrix.hpp"
#include "_zcovector-/_zcovector-zhematrix.hpp"
#include "_zcovector-/_zcovector-_zhematrix.hpp"
#include "_zcovector-/_zcovector-zgsmatrix.hpp"
#include "_zcovector-/_zcovector-_zgsmatrix.hpp"
#include "_zcovector-/_zcovector-zhsmatrix.hpp"
#include "_zcovector-/_zcovector-_zhsmatrix.hpp"
#include "_zcovector-/_zcovector-zhsmatrix.hpp"
#include "_zcovector-/_zcovector-_zhsmatrix.hpp"
#include "_zcovector-/_zcovector-double.hpp"
#include "_zcovector-/_zcovector-complex.hpp"

///////////////////////////////////////////////////////////
///////////////////////// double //////////////////////////
///////////////////////////////////////////////////////////
#include "double-/double-zcovector.hpp"
#include "double-/double-_zcovector.hpp"
#include "double-/double-zrovector.hpp"
#include "double-/double-_zrovector.hpp"
#include "double-/double-zgematrix.hpp"
#include "double-/double-_zgematrix.hpp"
#include "double-/double-zgbmatrix.hpp"
#include "double-/double-_zgbmatrix.hpp"
#include "double-/double-zhematrix.hpp"
#include "double-/double-_zhematrix.hpp"
#include "double-/double-zgsmatrix.hpp"
#include "double-/double-_zgsmatrix.hpp"
#include "double-/double-zhsmatrix.hpp"
#include "double-/double-_zhsmatrix.hpp"

///////////////////////////////////////////////////////////
///////////////////////// complex /////////////////////////
///////////////////////////////////////////////////////////
#include "complex-/complex-zcovector.hpp"
#include "complex-/complex-_zcovector.hpp"
#include "complex-/complex-zrovector.hpp"
#include "complex-/complex-_zrovector.hpp"
#include "complex-/complex-zgematrix.hpp"
#include "complex-/complex-_zgematrix.hpp"
#include "complex-/complex-zgbmatrix.hpp"
#include "complex-/complex-_zgbmatrix.hpp"
#include "complex-/complex-zhematrix.hpp"
#include "complex-/complex-_zhematrix.hpp"
#include "complex-/complex-zgsmatrix.hpp"
#include "complex-/complex-_zgsmatrix.hpp"
#include "complex-/complex-zhsmatrix.hpp"
#include "complex-/complex-_zhsmatrix.hpp"

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////
////////////////////////// small //////////////////////////
///////////////////////////////////////////////////////////
//// dcovector_small ////
#include "small/dcovector_small-constructors.hpp"
#include "small/dcovector_small-functions.hpp"
#include "small/dcovector_small-specialized.hpp"
//// drovector_small ////
#include "small/drovector_small-constructors.hpp"
#include "small/drovector_small-functions.hpp"
#include "small/drovector_small-specialized.hpp"
//// dgematrix_small ////
#include "small/dgematrix_small-constructors.hpp"
#include "small/dgematrix_small-functions.hpp"
#include "small/dgematrix_small-specialized.hpp"
//// dsymatrix_small ////
#include "small/dsymatrix_small-constructors.hpp"
#include "small/dsymatrix_small-functions.hpp"
#include "small/dsymatrix_small-specialized.hpp"
/////////////////////////////////////////////////
//// zcovector_small ////
#include "small/zcovector_small-constructors.hpp"
#include "small/zcovector_small-functions.hpp"
#include "small/zcovector_small-specialized.hpp"
//// zrovector_small ////
#include "small/zrovector_small-constructors.hpp"
#include "small/zrovector_small-functions.hpp"
#include "small/zrovector_small-specialized.hpp"
//// zgematrix_small ////
#include "small/zgematrix_small-constructors.hpp"
#include "small/zgematrix_small-functions.hpp"
#include "small/zgematrix_small-specialized.hpp"
//// zhematrix_small ////
#include "small/zhematrix_small-constructors.hpp"
#include "small/zhematrix_small-functions.hpp"
#include "small/zhematrix_small-specialized.hpp"
/////////////////////////////////////////////////
//// double ////
#include "small/double-small.hpp"
//// comple ////
#include "small/comple-small.hpp"

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
}//namespace CPPL
#endif//CPPLAPACK_H
