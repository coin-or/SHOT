//=============================================================================
/*! dsymatrix=_dsymatrix operator */
inline dsymatrix& dsymatrix::operator=(const _dsymatrix& mat)
{VERBOSE_REPORT;
  shallow_copy(mat);
  return *this;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! dsymatrix+=_dsymatrix operator */
inline dsymatrix& dsymatrix::operator+=(const _dsymatrix& mat)
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
  
  mat.destroy();
  return *this;
}

//=============================================================================
/*! dsymatrix-=_dsymatrix operator */
inline dsymatrix& dsymatrix::operator-=(const _dsymatrix& mat)
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
  
  mat.destroy();
  return *this;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! dsymatrix+_dsymatrix operator */
inline _dsymatrix operator+(const dsymatrix& matA, const _dsymatrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.n){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a summation." << std::endl
              << "Your input was (" << matA.n << "x" << matA.n << ") + (" << matB.n << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG

  for(long i=0; i<matA.n; i++){
    for(long j=0; j<=i; j++){
      matB.darray[j][i] +=matA.darray[j][i];
    }
  }
  
  return matB;
}

//=============================================================================
/*! dsymatrix-_dsymatrix operator */
inline _dsymatrix operator-(const dsymatrix& matA, const _dsymatrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.n){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a subtraction." << std::endl
              << "Your input was (" << matA.n << "x" << matA.n << ") - (" << matB.n << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG

  for(long i=0; i<matA.n; i++){
    for(long j=0; j<=i; j++){
      matB.darray[j][i] =matA.darray[j][i] -matB.darray[j][i];
    }
  }
  
  return matB;
}

//=============================================================================
/*! dsymatrix*_dsymatrix operator */
inline _dgematrix operator*(const dsymatrix& matA, const _dsymatrix& matB)
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
  dgematrix newmat( matA.n, matA.n );
  
  dgemm_( 'n', 'n', matA.n, matB.n, matA.n, 1.0, matA.array, matA.n,
          matB.array, matB.n, 0.0, newmat.array, matA.n );
  
  matB.destroy();
  return _(newmat);
}
