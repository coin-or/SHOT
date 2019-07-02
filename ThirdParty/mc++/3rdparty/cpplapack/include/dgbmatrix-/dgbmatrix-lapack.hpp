//=============================================================================
/*! solve A*X=Y using dgbsv\n
  The argument is dgematrix Y. Y is overwritten and become the solution X.
  A is also overwritten. */
inline long dgbmatrix::dgbsv(dgematrix& mat)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(m!=n || n!=mat.m){
    ERROR_REPORT;
    std::cerr << "These matrix and vector cannot be solved." << std::endl
              << "Your input was (" << m << "x" << n << ") and (" << mat.m << "x" << mat.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG 
  
  dgbmatrix newmat(m,n,kl,ku+kl);
  for(long i=0; i<m; i++){ for(long j=std::max(long(0),i-kl); j<std::min(n,i+ku+1); j++){
    newmat(i,j) =operator()(i,j);
  }}
  
  long NRHS(mat.n), LDAB(2*kl+ku+1),
    *IPIV(new long[n]), LDB(mat.m), INFO(1);
  dgbsv_(n, kl, ku, NRHS, newmat.array, LDAB, IPIV, mat.array, LDB, INFO);
  delete [] IPIV;
  
  swap(*this,newmat);
  
  if(INFO!=0){
    WARNING_REPORT;
    std::cerr << "Serious trouble happend. INFO = " << INFO << "." << std::endl;
  }
  return INFO;
}

//=============================================================================
/*! solve A*x=y using dgbsv\n
  The argument is dcovector y. y is overwritten and become the solution x.
  A is also overwritten. */
inline long dgbmatrix::dgbsv(dcovector& vec)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(m!=n || n!=vec.l){
    ERROR_REPORT;
    std::cerr << "These matrix and vector cannot be solved." << std::endl
              << "Your input was (" << m << "x" << n << ") and (" << vec.l << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG 
  
  dgbmatrix newmat(m,n,kl,ku+kl);
  for(long i=0; i<m; i++){ for(long j=std::max(long(0),i-kl); j<std::min(n,i+ku+1); j++){
    newmat(i,j) =operator()(i,j);
  }}
  
  long NRHS(1), LDAB(2*kl+ku+1),
    *IPIV(new long[n]), LDB(vec.l), INFO(1);
  dgbsv_(n, kl, ku, NRHS, newmat.array, LDAB, IPIV, vec.array, LDB, INFO);
  delete [] IPIV;

  swap(*this,newmat);
  
  if(INFO!=0){
    WARNING_REPORT;
    std::cerr << "Serious trouble happend. INFO = " << INFO << "." << std::endl;
  }
  return INFO;
}
