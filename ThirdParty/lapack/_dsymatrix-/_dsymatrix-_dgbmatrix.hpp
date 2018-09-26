//=============================================================================
/*! _dsymatrix+_dgbmatrix operator */
inline _dgematrix operator+(const _dsymatrix& matA, const _dgbmatrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.n || matA.n!=matB.m){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a summation." << std::endl
              << "Your input was (" << matA.n << "x" << matA.n << ") + (" << matB.m << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  dgematrix newmat(matA.n, matA.n);
  for(long i=0; i<matB.m; i++){
    for(long j=i; j<matA.n; j++){
      newmat(i,j) = newmat(j,i) = matA(i,j);
    }
    for(long j=std::max(long(0),i-matB.kl); j<std::min(matB.n,i+matB.ku+1); j++){
      newmat(i,j)+=matB(i,j);
    }
  }
  
  matA.destroy();
  matB.destroy();
  return _(newmat);
}

//=============================================================================
/*! _dsymatrix-_dgbmatrix operator */
inline _dgematrix operator-(const _dsymatrix& matA, const _dgbmatrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.n || matA.n!=matB.m){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a summation." << std::endl
              << "Your input was (" << matA.n << "x" << matA.n << ") + (" << matB.m << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  dgematrix newmat(matA.n, matA.n);
  for(long i=0; i<matB.m; i++){
    for(long j=i; j<matA.n; j++){
      newmat(i,j) = newmat(j,i) = matA(i,j);
    }
    for(long j=std::max(long(0),i-matB.kl); j<std::min(matB.n,i+matB.ku+1); j++){
      newmat(i,j) -= matB(i,j);
    }
  }

  matA.destroy();
  matB.destroy();
  return _(newmat);
}

//=============================================================================
/*! _dsymatrix*_dgbmatrix operator */
inline _dgematrix operator*(const _dsymatrix& matA, const _dgbmatrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.m){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a product." << std::endl
              << "Your input was (" << matA.n << "x" << matA.n << ") * (" << matB.m << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  dgematrix newmat( matA.n, matB.n );
  newmat.zero();
  
  for(long i=0; i<newmat.m; i++){
    for(long j=0; j<newmat.n; j++){
      for(long k=std::max(long(0),j-matB.ku); k<std::min(matB.m,j+matB.kl+1); k++){
        newmat(i,j) +=matA(i,k)*matB(k,j);
      }
    }
  }
  
  matA.destroy();
  matB.destroy();
  return _(newmat);
}
