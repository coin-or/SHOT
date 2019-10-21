//=============================================================================
/*! zgematrix+=_zgbmatrix operator */
inline zgematrix& zgematrix::operator+=(const _zgbmatrix& mat)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(n!=mat.n || m!=mat.m){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a summation." << std::endl
              << "Your input was (" << m << "x" << n << ") += (" << mat.m << "x" << mat.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  for(long i=0; i<mat.m; i++){
    for(long j=std::max(long(0),i-mat.kl); j<std::min(n,i+mat.ku+1); j++){
      operator()(i,j)+=mat(i,j);
    }
  }
  
  mat.destroy();
  return *this;
}

//=============================================================================
/*! zgematrix-=_zgbmatrix operator */
inline zgematrix& zgematrix::operator-=(const _zgbmatrix& mat)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(n!=mat.n || m!=mat.m){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a subtraction." << std::endl
              << "Your input was (" << m << "x" << n << ") -= (" << mat.m << "x" << mat.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  for(long i=0; i<mat.m; i++){
    for(long j=std::max(long(0),i-mat.kl); j<std::min(n,i+mat.ku+1); j++){
      operator()(i,j)-=mat(i,j);
    }
  }
  
  mat.destroy();
  return *this;
}
//=============================================================================
/*! zgematrix*=_zgbmatrix operator */
inline zgematrix& zgematrix::operator*=(const _zgbmatrix& mat)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(n!=mat.m){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a product." << std::endl
              << "Your input was (" << m << "x" << n << ") *= (" << mat.m << "x" << mat.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  zgematrix newmat(m,mat.n);
  newmat.zero();
  
  for(long i=0; i<newmat.m; i++){
    for(long j=0; j<newmat.n; j++){
      for(long k=std::max(long(0),j-mat.ku); k<std::min(mat.m,j+mat.kl+1); k++){
        newmat(i,j)+=operator()(i,k)*mat(k,j);
      }
    }
  }
  
  swap(*this,newmat);
  mat.destroy();
  return *this;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! zgematrix+_zgbmatrix operator */
inline _zgematrix operator+(const zgematrix& matA, const _zgbmatrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.n || matA.m!=matB.m){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a summation." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") + (" << matB.m << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  zgematrix newmat(matA);
  
  for(long i=0; i<matB.m; i++){
    for(long j=std::max(long(0),i-matB.kl); j<std::min(matB.n,i+matB.ku+1); j++){
      newmat(i,j)+=matB(i,j);
    }
  }
  
  matB.destroy();
  return _(newmat);
}

//=============================================================================
/*! zgematrix-_zgbmatrix operator */
inline _zgematrix operator-(const zgematrix& matA, const _zgbmatrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.n || matA.m!=matB.m){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a summation." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") + (" << matB.m << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  zgematrix newmat(matA);
  
  for(long i=0; i<matB.m; i++){
    for(long j=std::max(long(0),i-matB.kl); j<std::min(matB.n,i+matB.ku+1); j++){
      newmat(i,j)-=matB(i,j);
    }
  }
  
  matB.destroy();
  return _(newmat);
}

//=============================================================================
/*! zgematrix*_zgbmatrix operator */
inline _zgematrix operator*(const zgematrix& matA, const _zgbmatrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.m){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a product." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") * (" << matB.m << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  zgematrix newmat( matA.m, matB.n );
  newmat.zero();
  
  for(long i=0; i<newmat.m; i++){
    for(long j=0; j<newmat.n; j++){
      for(long k=std::max(long(0),j-matB.ku); k<std::min(matB.m,j+matB.kl+1); k++){
        newmat(i,j)+=matA(i,k)*matB(k,j);
      }
    }
  }
  
  matB.destroy();
  return _(newmat);
}
