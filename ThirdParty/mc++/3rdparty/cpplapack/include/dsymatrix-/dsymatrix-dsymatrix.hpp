//=============================================================================
/*! dsymatrix=dsymatrix operator */
inline dsymatrix& dsymatrix::operator=(const dsymatrix& mat)
{VERBOSE_REPORT;
  if(array!=mat.array){ // if it is NOT self substitution
    copy(mat);
  }
  return *this;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! dsymatrix+=dsymatrix operator */
inline dsymatrix& dsymatrix::operator+=(const dsymatrix& mat)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(n!=mat.n){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a summation." << std::endl
              << "Your input was (" << n << "x" << n << ") += (" << mat.n << "x" << mat.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  for(long i=0; i<n; i++){
    for(long j=0; j<=i; j++){
      darray[j][i] +=mat.darray[j][i];
    }
  }
  
  return *this;
}

//=============================================================================
/*! dsymatrix operator-= */
inline dsymatrix& dsymatrix::operator-=(const dsymatrix& mat)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(n!=mat.n){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a sutraction." << std::endl
              << "Your input was (" << n << "x" << n << ") -= (" << mat.n << "x" << mat.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  for(long i=0; i<n; i++){
    for(long j=0; j<=i; j++){
      darray[j][i] -=mat.darray[j][i];
    }
  }
  
  return *this;
}

//=============================================================================
/*! dsymatrix+dsymatrix operator */
inline _dsymatrix operator+(const dsymatrix& matA, const dsymatrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.n){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a summation." << std::endl
              << "Your input was (" << matA.n << "x" << matA.n << ") + (" << matB.n << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG

  long n = matA.n;
  dsymatrix newmat(n);
  for(long i=0; i<n; i++){
    for(long j=0; j<=i; j++){
      newmat.darray[j][i] =matA.darray[j][i] +matB.darray[j][i];
    }
  }
  
  return _(newmat);
}

//=============================================================================
/*! dsymatrix-dsymatrix operator */
inline _dsymatrix operator-(const dsymatrix& matA, const dsymatrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.n){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a subtraction." << std::endl
              << "Your input was (" << matA.n << "x" << matA.n << ") - (" << matB.n << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG

  long n = matA.n;
  dsymatrix newmat(n);
  for(long i=0; i<n; i++){
    for(long j=0; j<=i; j++){
      newmat.darray[j][i] =matA.darray[j][i] -matB.darray[j][i];
    }
  }

  return _(newmat);
}

//=============================================================================
/*! dsymatrix*dsymatrix operator */
inline _dgematrix operator*(const dsymatrix& matA, const dsymatrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.n){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a product." << std::endl
              << "Your input was (" << matA.n << "x" << matA.n << ") * (" << matB.n << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  matA.complete();
  matB.complete();
  dgematrix newmat(matA.n, matA.n);
  
  dgemm_( 'n', 'n', matA.n, matB.n, matA.n, 1.0, matA.array, matA.n,
          matB.array, matB.n, 0.0, newmat.array, matA.n );
  
  return _(newmat);
}
