//=============================================================================
/*! solve A*X=Y using zhesv\n
  The argument is dmatrix Y. Y is overwritten and become the solution X.
  A is also overwritten. 
*/
inline long zhematrix::zhesv(zgematrix& mat)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(n!=mat.n){
    ERROR_REPORT;
    std::cerr << "These two matrices cannot be solved." << std::endl
              << "Your input was (" << n << "x" << n << ") and (" << mat.n << "x" << mat.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  char UPLO('l');
  long NRHS(mat.n), LDA(n), *IPIV(new long[n]), LDB(mat.n), LWORK(-1), INFO(1);
  comple *WORK(new comple[1]);
  zhesv_(UPLO, n, NRHS, array, LDA, IPIV, mat.array, LDB, WORK, LWORK, INFO);
  
  INFO=1;
  LWORK = long(std::real(WORK[0]));
  delete [] WORK;  WORK =new comple[LWORK];
  zhesv_(UPLO, n, NRHS, array, LDA, IPIV, mat.array, LDB, WORK, LWORK, INFO);
  delete [] WORK; delete [] IPIV;
  
  if(INFO!=0){
    WARNING_REPORT;
    std::cerr << "Serious trouble happend. INFO = " << INFO << "." << std::endl;
  }
  return INFO;
}

//=============================================================================
/*! solve A*x=y using zhesv\n
  The argument is zcovector y. y is overwritten and become the solution x.
  A is also overwritten.
*/
inline long zhematrix::zhesv(zcovector& vec)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(n!=vec.l){
    ERROR_REPORT;
    std::cerr << "These matrix and vector cannot be solved." << std::endl
              << "Your input was (" << n << "x" << n << ") and (" << vec.l << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG 
  
  char UPLO('l');
  long NRHS(1), LDA(n), *IPIV(new long[n]), LDB(vec.l), LWORK(-1), INFO(1);
  comple *WORK( new comple[1] );
  zhesv_(UPLO, n, NRHS, array, LDA, IPIV, vec.array, LDB, WORK, LWORK, INFO);
  
  INFO=1;
  LWORK = long(std::real(WORK[0]));
  delete [] WORK;  WORK = new comple[LWORK];
  zhesv_(UPLO, n, NRHS, array, LDA, IPIV, vec.array, LDB, WORK, LWORK, INFO);
  delete [] WORK;  delete [] IPIV;
  
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
/*! calculate eigenvalues and eigenvectors.\n
  All of the arguments need not to be initialized.
  w is overwitten and become eigenvalues.
  This matrix is also overwritten. 
  if jobz=1, this matrix becomes eigenvectors.
*/
inline long zhematrix::zheev(std::vector<double>& w,
                             const bool& jobz=0)
{VERBOSE_REPORT;
  w.resize(n);
  char JOBZ, UPLO('l');
  if(jobz==0){ JOBZ='n'; } else{ JOBZ='V'; }
  long LDA(n), INFO(1), LWORK(-1);
  double *RWORK(new double[std::max(long(1), 3*n-2)]);
  comple *WORK(new comple[1]);
  zheev_(JOBZ, UPLO, n, array, LDA, &w[0], WORK, LWORK, RWORK, INFO);
  
  INFO=1;
  LWORK = long(std::real(WORK[0]));
  delete [] WORK;  WORK = new comple[LWORK];
  zheev_(JOBZ, UPLO, n, array, LDA, &w[0], WORK, LWORK, RWORK, INFO);
  delete [] RWORK;  delete [] WORK;
  
  if(INFO!=0){
    WARNING_REPORT;
    std::cerr << "Serious trouble happend. INFO = " << INFO << "." << std::endl;
  }
  return INFO;
}

//=============================================================================
/*! calculate eigenvalues and eigenvectors.\n
  All of the arguments need not to be initialized.
  w and v are overwitten and become 
  eigenvalues and eigenvectors, respectively.
  This matrix is also overwritten. 
*/
inline long zhematrix::zheev(std::vector<double>& w,
                             std::vector<zcovector>& v)
{VERBOSE_REPORT;
  w.resize(n);  v.resize(n);
  for(long i=0; i<n; i++){ v[i].resize(n); }
  char JOBZ('V'), UPLO('l');
  long LDA(n), INFO(1), LWORK(-1);
  double *RWORK(new double[std::max(long(1), 3*n-2)]);
  comple *WORK(new comple[1]);
  zheev_(JOBZ, UPLO, n, array, LDA, &w[0], WORK, LWORK, RWORK, INFO);
  
  INFO=1;
  LWORK = long(std::real(WORK[0]));
  delete [] WORK;  WORK = new comple[LWORK];
  zheev_(JOBZ, UPLO, n, array, LDA, &w[0], WORK, LWORK, RWORK, INFO);
  delete [] RWORK;  delete [] WORK;
  
  //// forming ////
  for(long i=0; i<n; i++){ for(long j=0; j<n; j++){
    v[j](i) = array[i+n*j];
  }}
  
  if(INFO!=0){
    WARNING_REPORT;
    std::cerr << "Serious trouble happend. INFO = " << INFO << "." << std::endl;
  }
  return INFO;
}

//=============================================================================
/*! calculate eigenvalues and eigenvectors.\n
  All of the arguments need not to be initialized.
  w and v are overwitten and become 
  eigenvalues and eigenvectors, respectively.
  This matrix is also overwritten. 
*/
inline long zhematrix::zheev(std::vector<double>& w,
                             std::vector<zrovector>& v)
{VERBOSE_REPORT;
  w.resize(n);  v.resize(n);
  for(long i=0; i<n; i++){ v[i].resize(n); }
  char JOBZ('V'), UPLO('l');
  long LDA(n), INFO(1), LWORK(-1);
  double *RWORK(new double[std::max(long(1), 3*n-2)]);
  comple *WORK(new comple[1]);
  zheev_(JOBZ, UPLO, n, array, LDA, &w[0], WORK, LWORK, RWORK, INFO);
  
  INFO=1;
  LWORK = long(std::real(WORK[0]));
  delete [] WORK;  WORK = new comple[LWORK];
  zheev_(JOBZ, UPLO, n, array, LDA, &w[0], WORK, LWORK, RWORK, INFO);
  delete [] RWORK;  delete [] WORK;
  
  //// forming ////
  for(long i=0; i<n; i++){ for(long j=0; j<n; j++){
    v[j](i) = array[i+n*j];
  }}
  
  if(INFO!=0){
    WARNING_REPORT;
    std::cerr << "Serious trouble happend. INFO = " << INFO << "." << std::endl;
  }
  return INFO;
}
