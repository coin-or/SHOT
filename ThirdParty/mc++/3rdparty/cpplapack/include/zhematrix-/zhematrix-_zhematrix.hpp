//=============================================================================
/*! zhematrix=_zhematrix operator */
inline zhematrix& zhematrix::operator=(const _zhematrix& mat)
{VERBOSE_REPORT;
  shallow_copy(mat);
  return *this;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! zhematrix+=_zhematrix operator */
inline zhematrix& zhematrix::operator+=(const _zhematrix& mat)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(n!=mat.n){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a summation." << std::endl
              << "Your input was (" << n << "x" << n << ") += (" << mat.n << "x" << mat.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  for(long i=0; i<n; i++){ for(long j=0; j<=i; j++){
    array[i+n*j]+=mat.array[i+n*j];
  }}
  
  mat.destroy();
  return *this;
}

//=============================================================================
/*! zhematrix-=_zhematrix operator */
inline zhematrix& zhematrix::operator-=(const _zhematrix& mat)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(n!=mat.n){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a sutraction." << std::endl
              << "Your input was (" << n << "x" << n << ") -= (" << mat.n << "x" << mat.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  for(long i=0; i<n; i++){ for(long j=0; j<=i; j++){
    array[i+n*j]-=mat.array[i+n*j];
  }}
  
  mat.destroy();
  return *this;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! zhematrix+_zhematrix operator */
inline _zhematrix operator+(const zhematrix& matA, const _zhematrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.n){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a summation." << std::endl
              << "Your input was (" << matA.n << "x" << matA.n << ") + (" << matB.n << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG

  for(long i=0; i<matA.n; i++){ for(long j=0; j<=i; j++){
    matB.array[i+matA.n*j]+=matA.array[i+matA.n*j];
  }}
  
  return matB;
}

//=============================================================================
/*! zhematrix-_zhematrix operator */
inline _zhematrix operator-(const zhematrix& matA, const _zhematrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.n){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a subtraction." << std::endl
              << "Your input was (" << matA.n << "x" << matA.n << ") - (" << matB.n << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  for(long i=0; i<matA.n; i++){ for(long j=0; j<=i; j++){
    matB.array[i] =matA.array[i]-matB.array[i];
  }}
  
  return matB;
}

//=============================================================================
/*! zhematrix*_zhematrix operator */
inline _zgematrix operator*(const zhematrix& matA, const _zhematrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.n){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a product." << std::endl
              << "Your input was (" << matA.n << "x" << matA.n << ") * (" << matB.n << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  matB.complete();
  zgematrix newmat( matA.n, matB.n );
  
  zhemm_( 'l', 'l', matA.n, matB.n, comple(1.0,0.0), 
          matA.array, matA.n, matB.array, matB.n, 
          comple(0.0,0.0), newmat.array, newmat.m );
  
  matB.destroy();
  return _(newmat);
}
