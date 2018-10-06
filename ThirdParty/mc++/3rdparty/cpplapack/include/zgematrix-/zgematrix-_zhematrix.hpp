//=============================================================================
/*! zgematrix+=_zhematrix operator */
inline zgematrix& zgematrix::operator+=(const _zhematrix& mat)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(n!=mat.n || m!=mat.n){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a summation." << std::endl
              << "Your input was (" << m << "x" << n << ") += (" << mat.n << "x" << mat.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  for(long i=0; i<m; i++){ for(long j=0; j<n; j++){
    operator()(i,j) +=mat(i,j);
  }}
  
  mat.destroy();
  return *this;
}

//=============================================================================
/*! zgematrix-=_zhematrix operator */
inline zgematrix& zgematrix::operator-=(const _zhematrix& mat)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(n!=mat.n || m!=mat.n){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a sutraction." << std::endl
              << "Your input was (" << m << "x" << n << ") -= (" << mat.n << "x" << mat.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  for(long i=0; i<m; i++){ for(long j=0; j<n; j++){
    operator()(i,j) -=mat(i,j);
  }}
  
  mat.destroy();
  return *this;
}

//=============================================================================
/*! zgematrix*=_zhematrix operator */
inline zgematrix& zgematrix::operator*=(const _zhematrix& mat)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(n!=mat.n){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a product." << std::endl
              << "Your input was (" << m << "x" << n << ") *= (" << mat.n << "x" << mat.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  zgematrix newmat( m, mat.n );
  
  zhemm_( 'R', 'l', mat.n, n, comple(1.0,0.0), mat.array, mat.n, 
          array, m, comple(0.0,0.0), newmat.array, newmat.m );
  
  swap(*this,newmat);
  mat.destroy();
  return *this;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! zgematrix+_zhematrix operator */
inline _zgematrix operator+(const zgematrix& matA, const _zhematrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.n || matA.m!=matB.n){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a summation." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") + (" << matB.n << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  zgematrix newmat(matA);
  for(long i=0; i<matA.m; i++){ for(long j=0; j<matA.n; j++){
    newmat(i,j) +=matB(i,j);
  }}
  
  matB.destroy();
  return _(newmat);
}

//=============================================================================
/*! zgematrix-_zhematrix operator */
inline _zgematrix operator-(const zgematrix& matA, const _zhematrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.n || matA.m!=matB.n){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a subtraction." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") - (" << matB.n << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  zgematrix newmat(matA);
  for(long i=0; i<matA.m; i++){ for(long j=0; j<matA.n; j++){
    newmat(i,j) -=matB(i,j);
  }}
  
  matB.destroy();
  return _(newmat);
}

//=============================================================================
/*! zgematrix*_zhematrix operator */
inline _zgematrix operator*(const zgematrix& matA, const _zhematrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.n){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a product." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") * (" << matB.n << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  zgematrix newmat( matA.n, matB.n );
  
  zhemm_( 'R', 'l', matB.n, matA.n, comple(1.0,0.0), 
          matB.array, matB.n, matA.array, matA.m, 
          comple(0.0,0.0), newmat.array, newmat.m );
  
  matB.destroy();
  return _(newmat);
}
