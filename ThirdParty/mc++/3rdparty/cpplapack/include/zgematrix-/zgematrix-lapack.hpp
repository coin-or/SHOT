//=============================================================================
/*! solve A*X=Y using zgesv\n
  The argument is zgematrix Y. Y is overwritten and become the solution X.
  A is also overwritten and become P*l*U. */
inline long zgematrix::zgesv(zgematrix& mat)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(m!=n || m!=mat.m){
    ERROR_REPORT;
    std::cerr << "These two matrices cannot be solved." << std::endl
              << "Your input was (" << m << "x" << n << ") and (" << mat.m << "x" << mat.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG 
  
  long NRHS(mat.n), LDA(n), *IPIV(new long[n]), LDB(mat.m), INFO(1);
  zgesv_(n, NRHS, array, LDA, IPIV, mat.array, LDB, INFO);
  delete [] IPIV;
  
  if(INFO!=0){
    WARNING_REPORT;
    std::cerr << "Serious trouble happend. INFO = " << INFO << "." << std::endl;
  }
  return INFO;
}

//=============================================================================
/*! solve A*x=y using zgesv\n
  The argument is zcovector y. y is overwritten and become the solution x.
  A is also overwritten and become P*l*U. */
inline long zgematrix::zgesv(zcovector& vec)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(m!=n || m!=vec.l){
    ERROR_REPORT;
    std::cerr << "These matrix and vector cannot be solved." << std::endl
              << "Your input was (" << m << "x" << n << ") and (" << vec.l << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG 
  long NRHS(1), LDA(n), *IPIV(new long[n]), LDB(vec.l), INFO(1);
  zgesv_(n, NRHS, array, LDA, IPIV, vec.array, LDB, INFO);
  delete [] IPIV;
  
  if(INFO!=0){
    WARNING_REPORT;
    std::cerr << "Serious trouble happend. INFO = " << INFO << "." << std::endl;
  }
  return INFO;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! solve overdetermined or underdetermined A*X=Y using zgels\n*/
inline long zgematrix::zgels(zgematrix& mat)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(m!=mat.m){
    ERROR_REPORT;
    std::cerr << "These two matrices cannot be solved." << std::endl
              << "Your input was (" << m << "x" << n << ") and (" << mat.m << "x" << mat.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG    
  
  if(m<n){ //underdetermined
    zgematrix tmp(n,mat.n);
    for(long i=0; i<mat.m; i++){ for(long j=0; j<mat.n; j++){
      tmp(i,j) =mat(i,j);
    }}
    mat.clear();
    swap(mat,tmp);
  }
  
  char TRANS('n');
  long NRHS(mat.n), LDA(m), LDB(mat.m),
    LWORK(std::min(m,n)+std::max(std::min(m,n),NRHS)), INFO(1);
  comple *WORK(new comple[LWORK]);
  zgels_(TRANS, m, n, NRHS, array, LDA, mat.array, LDB, WORK, LWORK, INFO);
  delete [] WORK;
  
  if(m>n){ //overdetermined
    zgematrix tmp(n,mat.n);
    for(long i=0; i<tmp.m; i++){ for(long j=0; j<tmp.n; j++){
      tmp(i,j) =mat(i,j);
    }}
    mat.clear();
    swap(mat,tmp);
  }
  
  if(INFO!=0){
    WARNING_REPORT;
    std::cerr << "Serious trouble happend. INFO = " << INFO << "." << std::endl;
  }
  return INFO;
}

//=============================================================================
/*! solve overdetermined or underdetermined A*x=y using zgels\n*/
inline long zgematrix::zgels(zcovector& vec)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(m!=vec.l){
    ERROR_REPORT;
    std::cerr << "These matrix and vector cannot be solved." << std::endl
              << "Your input was (" << m << "x" << n << ") and (" << vec.l << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG    
  
  if(m<n){ //underdetermined
    zcovector tmp(n);
    for(long i=0; i<vec.l; i++){ tmp(i)=vec(i); }
    vec.clear();
    swap(vec,tmp);
  }
  
  char TRANS('n');
  long NRHS(1), LDA(m), LDB(vec.l),
    LWORK(std::min(m,n)+std::max(std::min(m,n),NRHS)), INFO(1);
  comple *WORK(new comple[LWORK]);
  zgels_(TRANS, m, n, NRHS, array, LDA, vec.array, LDB, WORK, LWORK, INFO);
  delete [] WORK;
  
  if(m>n){ //overdetermined
    zcovector tmp(n);
    for(long i=0; i<tmp.l; i++){ tmp(i)=vec(i); }
    vec.clear();
    swap(vec,tmp);
  }
  
  if(INFO!=0){
    WARNING_REPORT;
    std::cerr << "Serious trouble happend. INFO = " << INFO << "." << std::endl;
  }
  return INFO;
}

//=============================================================================
/*! solve overdetermined or underdetermined A*X=Y using zgels
  with the sum of residual squares output\n
  The residual is set as the columnwise sum of residual squares 
  for overdetermined problems 
  while it is always zero for underdetermined problems.
*/
inline long zgematrix::zgels(zgematrix& mat, drovector& residual)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(m!=mat.m){
    ERROR_REPORT;
    std::cerr << "These two matrices cannot be solved." << std::endl
              << "Your input was (" << m << "x" << n << ") and (" << mat.m << "x" << mat.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  residual.resize(mat.n); residual.zero();
  
  if(m<n){ //underdetermined
    zgematrix tmp(n,mat.n);
    for(long i=0; i<mat.m; i++){ for(long j=0; j<mat.n; j++){
      tmp(i,j) =mat(i,j);
    }}
    mat.clear();
    swap(mat,tmp);
  }
  
  char TRANS('n');
  long NRHS(mat.n), LDA(m), LDB(mat.m),
    LWORK(std::min(m,n)+std::max(std::min(m,n),NRHS)), INFO(1);
  comple *WORK(new comple[LWORK]);
  zgels_(TRANS, m, n, NRHS, array, LDA, mat.array, LDB, WORK, LWORK, INFO);
  delete [] WORK;
  
  if(m>n){ //overdetermined
    for(long i=0; i<residual.l; i++){ for(long j=0; j<m-n; j++){
      residual(i) += std::norm(mat(n+j,i));
    }}
    
    zgematrix tmp(n,mat.n);
    for(long i=0; i<tmp.m; i++){ for(long j=0; j<tmp.n; j++){
      tmp(i,j) =mat(i,j);
    }}
    mat.clear();
    swap(mat,tmp);
  }
  
  if(INFO!=0){
    WARNING_REPORT;
    std::cerr << "Serious trouble happend. INFO = " << INFO << "." << std::endl;
  }
  return INFO;
}

//=============================================================================
/*! solve overdetermined or underdetermined A*x=y using zgels
  with the sum of residual squares output\n
  The residual is set as the sum of residual squares for overdetermined problems
  while it is always zero for underdetermined problems.
*/
inline long zgematrix::zgels(zcovector& vec, double& residual)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(m!=vec.l){
    ERROR_REPORT;
    std::cerr << "These matrix and vector cannot be solved." << std::endl
              << "Your input was (" << m << "x" << n << ") and (" << vec.l << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG    
  
  residual=0.0;
  
  if(m<n){ //underdetermined
    zcovector tmp(n);
    for(long i=0; i<vec.l; i++){ tmp(i)=vec(i); }
    vec.clear();
    swap(vec,tmp);
  }
  
  char TRANS('n');
  long NRHS(1), LDA(m), LDB(vec.l),
    LWORK(std::min(m,n)+std::max(std::min(m,n),NRHS)), INFO(1);
  comple *WORK(new comple[LWORK]);
  zgels_(TRANS, m, n, NRHS, array, LDA, vec.array, LDB, WORK, LWORK, INFO);
  delete [] WORK;
  
  if(m>n){ //overdetermined
    for(long i=0; i<m-n; i++){ residual+=std::norm(vec(n+i)); }
    
    zcovector tmp(n);
    for(long i=0; i<tmp.l; i++){ tmp(i)=vec(i); }
    vec.clear();
    swap(vec,tmp);
  }
  
  if(INFO!=0){
    WARNING_REPORT;
    std::cerr << "Serious trouble happend. INFO = " << INFO << "." << std::endl;
  }
  return INFO;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! calculate the least-squares-least-norm solution 
  for overdetermined or underdetermined A*x=y using zgelss\n */
inline long zgematrix::zgelss(zcovector& B, dcovector& S, long& RANK,
                              const double RCOND =-1. )
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(m!=B.l){
    ERROR_REPORT;
    std::cerr << "These matrix and vector cannot be solved." << std::endl
              << "Your input was (" << m << "x" << n << ") and (" << B.l << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG    
  
  if(m<n){ //underdetermined
    zcovector tmp(n);
    for(long i=0; i<B.l; i++){ tmp(i)=B(i); }
    B.clear();
    swap(B,tmp);
  }
  
  S.resize(std::min(m,n));
  
  long NRHS(1), LDA(m), LDB(B.l),
    LWORK(2*std::min(m,n) +std::max(std::max(m,n),NRHS)), INFO(1);
  double *RWORK(new double[5*std::min(m,n)]);
  comple *WORK(new comple[LWORK]);
  zgelss_(m, n, NRHS, array, LDA, B.array, LDB, S.array, RCOND, RANK, 
          WORK, LWORK, RWORK, INFO);
  delete [] RWORK; delete [] WORK;
  
  if(INFO!=0){
    WARNING_REPORT;
    std::cerr << "Serious trouble happend. INFO = " << INFO << "." << std::endl;
  }
  return INFO;
}

//=============================================================================
/*! calculate the least-squares-least-norm solution 
  for overdetermined or underdetermined A*x=y using zgelss\n */
inline long zgematrix::zgelss(zgematrix& B, dcovector& S, long& RANK,
                              const double RCOND =-1. )
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(m!=B.m){
    ERROR_REPORT;
    std::cerr << "These matrix and vector cannot be solved." << std::endl
              << "Your input was (" << m << "x" << n << ") and (" << B.m << "x" << B.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG    
  
  if(m<n){ //underdetermined
    zgematrix tmp(n,B.n);
    for(long i=0; i<B.m; i++){
      for(long j=0; j<B.n; j++){
        tmp(i,j)=B(i,j);
      }
    }
    B.clear();
    swap(B,tmp);
  }
  
  S.resize(std::min(m,n));
  
  long NRHS(B.n), LDA(m), LDB(B.m),
    LWORK(2*std::min(m,n) +std::max(std::max(m,n),NRHS)), INFO(1);
  double *RWORK(new double[5*std::min(m,n)]);
  comple *WORK(new comple[LWORK]);
  zgelss_(m, n, NRHS, array, LDA, B.array, LDB, S.array, RCOND, RANK, 
          WORK, LWORK, RWORK, INFO);
  delete [] RWORK; delete [] WORK;
  
  if(INFO!=0){
    WARNING_REPORT;
    std::cerr << "Serious trouble happend. INFO = " << INFO << "." << std::endl;
  }
  return INFO;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! calculate eigenvalues\n
  The argument need not to be initialized.
  w is overwitten and become eigenvalues.
  This matrix is also overwritten. */
inline long zgematrix::zgeev(std::vector< comple >& w)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(m!=n){
    ERROR_REPORT;
    std::cerr << "This matrix cannot have eigenvalues." << std::endl
              << "Your input was (" << m << "x" << n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  w.resize(n);
  char JOBVL('n'), JOBVR('n');
  long LDA(n), LDVL(1), LDVR(1), LWORK(4*n), INFO(1);
  double *RWORK(new double[2*n]);
  comple *VL(NULL), *VR(NULL), 
    *WORK(new comple[LWORK]);
  zgeev_(JOBVL, JOBVR, n, array, LDA, &w[0], 
         VL, LDVL, VR, LDVR, WORK, LWORK, RWORK, INFO);
  delete [] RWORK; delete [] WORK; delete [] VL; delete [] VR;
  
  if(INFO!=0){
    WARNING_REPORT;
    std::cerr << "Serious trouble happend. INFO = " << INFO << "." << std::endl;
  }
  return INFO;
}

//=============================================================================
/*! calculate eigenvalues and right eigenvectors\n
  All of the arguments need not to be initialized.
  w, vr are overwitten and become eigenvalues and right eigenvectors, 
  respectively. 
  This matrix is also overwritten. */
inline long zgematrix::zgeev(std::vector< comple >& w,
                             std::vector<zcovector>& vr)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(m!=n){
    ERROR_REPORT;
    std::cerr << "This matrix cannot have eigenvalues." << std::endl
              << "Your input was (" << m << "x" << n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  w.resize(n);  vr.resize(n);
  for(long i=0; i<n; i++){ vr[i].resize(n); }
  zgematrix VR(n,n);
  char JOBVL('n'), JOBVR('V');
  long LDA(n), LDVL(1), LDVR(n), LWORK(4*n), INFO(1);
  double *RWORK(new double[2*n]);
  comple *VL(NULL), *WORK(new comple[LWORK]);
  zgeev_(JOBVL, JOBVR, n, array, LDA, &w[0], 
         VL, LDVL, VR.array, LDVR, WORK, LWORK, RWORK, INFO);
  delete [] RWORK; delete [] WORK; delete [] VL;
  
  //// forming ////
  for(long j=0; j<n; j++){  for(long i=0; i<n; i++){
    vr[j](i) = VR(i,j);
  }}
  
  if(INFO!=0){
    WARNING_REPORT;
    std::cerr << "Serious trouble happend. INFO = " << INFO << "." << std::endl;
  }
  return INFO;
}

//=============================================================================
/*! calculate eigenvalues and left eigenvectors\n
  All of the arguments need not to be initialized.
  w, vr are overwitten and become eigenvalues and left eigenvectors, 
  respectively. 
  This matrix is also overwritten. */
inline long zgematrix::zgeev(std::vector< comple >& w,
                             std::vector<zrovector>& vl)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(m!=n){
    ERROR_REPORT;
    std::cerr << "This matrix cannot have eigenvalues." << std::endl
              << "Your input was (" << m << "x" << n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  w.resize(n);  vl.resize(n);
  for(long i=0; i<n; i++){ vl[i].resize(n); }
  zgematrix VL(n,n);
  char JOBVL('V'), JOBVR('n');
  long LDA(n), LDVL(n), LDVR(1), LWORK(4*n), INFO(1);
  double *RWORK(new double[2*n]);
  comple *VR(NULL), *WORK(new comple[LWORK]);
  zgeev_(JOBVL, JOBVR, n, array, LDA, &w[0], 
         VL.array, LDVL, VR, LDVR, WORK, LWORK, RWORK, INFO);
  delete [] RWORK; delete [] WORK; delete [] VR;
  
  //// forming ////
  for(long j=0; j<n; j++){ for(long i=0; i<n; i++){
    vl[j](i) = std::conj(VL(i,j));
  }}
  
  if(INFO!=0){
    WARNING_REPORT;
    std::cerr << "Serious trouble happend. INFO = " << INFO << "." << std::endl;
  }
  return INFO;
}


///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
//inline long zgematrix::zgegv()


///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! compute the singular value decomposition (SVD)\n
  The arguments are zcocector S, zgematrix U and VT.
  All of them need not to be initialized.
  S, U and VT are overwitten and become singular values, left singular vectors,
  and right singular vectors respectively.
  This matrix also overwritten.
*/
inline long zgematrix::zgesvd(dcovector& S, zgematrix& U, zgematrix& VT)
{VERBOSE_REPORT;
  char JOBU('A'), JOBVT('A');
  long LDA(m), LDU(m), LDVT(n),
    LWORK(std::max(3*std::min(m,n)+std::max(m,n),5*std::min(m,n))), INFO(1);
  double *RWORK(new double[5*std::min(m,n)]);
  comple *WORK(new comple[LWORK]);
  S.resize(std::min(m,n)); U.resize(LDU,m); VT.resize(LDVT,n);
  
  zgesvd_(JOBU, JOBVT, m, n, array, LDA, S.array, U.array, 
          LDU, VT.array, LDVT, WORK, LWORK, RWORK, INFO);
  delete [] RWORK; delete [] WORK;
  
  if(INFO!=0){
    WARNING_REPORT;
    std::cerr << "Serious trouble happend. INFO = " << INFO << "." << std::endl;
  }
  return INFO;
}
