// Copyright (C) 2009-2016 Benoit Chachuat, Imperial College London.
// All Rights Reserved.
// This code is published under the Eclipse Public License.

#ifndef MC__MCLAPACK_H
#define MC__MCLAPACK_H

#include <iostream>
#include <iomanip>
#include <string>

#undef  MC__DEBUG_EIGEN
#undef  MC__DEBUG_DGEQRF

//extern "C" void dsyev_
//( const char*jobz, const char*uplo, const unsigned int*n, double*a,
//  const unsigned int*lda, double*w, double*work, const int*lwork, int*info );
extern "C" void dsyev_
( const char&, const char&, const long int&, double*, const long int&,
  double*, double*, const long int&, long int& );
extern "C" void dgeqrf_
  ( const int*, const int*, double*, const int*, double*, double*,
    const int*, int* );

namespace mc
{
 
inline void pause()
{
  int tmp;
  std::cout << "ENTER <1> TO CONTINUE" << std::endl;
  std::cin  >> tmp;
}

template< typename U > inline void display
( const unsigned int m, const unsigned int n, const U*a, const unsigned int lda,
  const std::string&stra, std::ostream&os )
{
  os << stra << " =" << std::endl << std::scientific
     << std::setprecision(5);
  for( unsigned int im=0; a && im<m; im++ ){
    for( unsigned int in=0; in<n; in++ ){
      os << a[in*lda+im] << "  ";
    }
    os << std::endl;
  }
  os << std::endl;

  //if( os.rdbuf() == std::cout.rdbuf() || os.rdbuf() == std::cerr.rdbuf() ) pause();
}

//! @brief Wrapper to LAPACK function <tt>_dsyev<tt> doing eigenvalue decomposition of symmetric <tt>n</tt>-by-<tt>n</tt> matrix <tt>A</tt>. A pointer to an array containing the eignenvalues is returned - NULL pointer if the eigenvalue decomposition was unsuccessful. If <tt>eigv</tt> is set to <tt>true</tt> eigenvector too are computed and returned in <tt>A</tt>, which is therefore alterred. Note that <tt>A</tt> is altered even if eigenvector are not computed.
inline double* dsyev_wrapper
( const unsigned int n, double*A, const bool eigv=false )
{
  long info(1);
  double*D = new double[n];
#ifdef MC__DEBUG_EIGEN
  display( n, n, A, n, "Matrix A", std::cout );
#endif

  // get optimal size
  char JOBZ = (eigv?'V':'N'), UPLO = 'U';
  double worktmp;
  int lwork = -1;
  dsyev_( JOBZ, UPLO, n, A, n, D, &worktmp, lwork, info );

  // perform eigenvalue decomposition
  lwork = (int)worktmp;
  double*work = new double[lwork];
  dsyev_( JOBZ, UPLO, n, A, n, D, work, lwork, info );
#ifdef MC__DEBUG_EIGEN
  if( eigv ) TModel<T>::_display( n, n, A, n, "Matrix U", std::cout );
  display( 1, n, D, 1, "Matrix D", std::cout );
#endif
  delete[] work;

#ifdef MC__DEBUG_EIGEN
  std::cout << "INFO: " << info << std::endl;
  pause();
#endif
  if( info ){ delete[] D; return 0; }
  return D;
}

} // namespace mc

#include "cpplapack.h"

namespace CPPL{ //!< namespace for CPPLapack

//=============================================================================
/*! calculate inverse.\n
  All of the arguments need not to be initialized.
  mat is not overwritten. 
*/
inline long dsysv( const dsymatrix& mat, dsymatrix& mat_inv )
{ 
  VERBOSE_REPORT;
  dsymatrix mat_cp(mat);
  mat_inv.resize(mat.n);
  mat_inv.identity(); 
  char UPLO('l');
  long NRHS(mat.n), LDA(mat.n), *IPIV(new long[mat.n]), LDB(mat.n), LWORK(-1), INFO(1);
  double *WORK( new double[1] );
  dsysv_(UPLO, mat.n, NRHS, mat_cp.array, LDA, IPIV, mat_inv.array, LDB, WORK, LWORK, INFO);

  LWORK = long(WORK[0]);
  delete [] WORK;  WORK = new double[LWORK];
  dsysv_(UPLO, mat.n, NRHS, mat_cp.array, LDA, IPIV, mat_inv.array, LDB, WORK, LWORK, INFO);
  delete [] WORK; delete [] IPIV;

  if(INFO!=0){
    WARNING_REPORT;
    std::cerr << "DSYSV: Serious trouble happend. INFO = " << INFO << "." << std::endl;
  } 
  return INFO;
}

/*! calculate QR decomposition.\n
  All of the arguments need not to be initialized.
  mat is not overwritten. 
*/
inline long dgeqrf( const dgematrix& Amat, dgematrix& Qmat, dgematrix& Rmat )
{ 
  VERBOSE_REPORT;
  dgematrix A(Amat);
  const int M(Amat.m), N(Amat.n), NM = (M<N?M:N);
  dcovector TAU(NM), WORK(M);

  // Get optimal size for QR decomposition
  int LWORK(-1), INFO(0);
  dgeqrf_(&M, &N, A.array, &N, TAU.array, WORK.array, &LWORK, &INFO);
  LWORK = (int)*WORK.array;
  WORK.resize(LWORK);

  // Perform QR decomposition
  dgeqrf_(&M, &N, A.array, &N, TAU.array, WORK.array, &LWORK, &INFO);
  if( INFO ){
    WARNING_REPORT;
    std::cerr << "DGEQRF: Serious trouble happend. INFO = " << INFO << "." << std::endl;
    return INFO;
  }

  // Post-process QR decomposition results for R
  Rmat.resize(M,N); Rmat.zero();
  for( int ir=0; ir<NM; ir++ )
    for(int ic=ir; ic<N; ic++ )
      Rmat(ir,ic) = A(ir,ic);
#ifdef MC__DEBUG_DGEQRF
  std::cout << "Matrix R:" << Rmat << std::endl;
#endif

  // Post-process QR decomposition results for Q
  Qmat.resize(M,N);
  Qmat.identity();
  dcovector V(M); V.zero();
  dgematrix H(M,M);

  for( int im=0; im<NM; im++ ){
    V(im) = 1.; if( im ) V(im-1) = 0.;
    for( int ir=im+1; ir<M; ir++ ) V(ir) = A(ir,im);
    H.identity(); H -= TAU(im) * V * t(V);
    Qmat *= H;
  }
#ifdef MC__DEBUG_DGEQRF
  std::cout << "Matrix Q:" << Qmat << std::endl;
  std::cout << "Check Q*R:" << Qmat*Rmat << std::endl;
  std::cout << "=? A:" << Amat << std::endl;
#endif
  return INFO;
}

inline long dgesv( const dgematrix& mat, dgematrix& mat_inv )
{
  VERBOSE_REPORT;
  if(mat.m!=mat.n){
    ERROR_REPORT;
    std::cerr << "This matrix is not square and has no inverse matrix." << std::endl
              << "Your input was (" << mat.m << "x" << mat.n << ")." << std::endl;
    return -1;
  }

  dgematrix mat_cp(mat);
  mat_inv.resize(mat.m,mat.n);
  mat_inv.identity(); 
  return mat_cp.dgesv( mat_inv );
}

inline long dgbsv( const dgbmatrix& mat, dgematrix& mat_inv )
{
  VERBOSE_REPORT;
  if(mat.m!=mat.n){
    ERROR_REPORT;
    std::cerr << "This matrix is not square and has no inverse matrix." << std::endl
              << "Your input was (" << mat.m << "x" << mat.n << ")." << std::endl;
    return -1;
  }

  dgbmatrix mat_cp(mat);
  mat_inv.resize(mat.m,mat.n);
  mat_inv.identity(); 
  return mat_cp.dgbsv( mat_inv );
}

}//namespace CPPL

#endif
