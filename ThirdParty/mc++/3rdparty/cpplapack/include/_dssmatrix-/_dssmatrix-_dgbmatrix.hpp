//=============================================================================
/*! _dgsmatrix+_dgbmatrix operator */
/*
inline _dgematrix operator+(const _dgsmatrix& matA, const _dgbmatrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.n || matA.m!=matB.m){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a summation." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") + (" << matB.m << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  dgematrix newmat(matA);
  
  for(long i=0; i<matB.m; i++){
    for(long j=max(0,i-matB.kl); j<min(matB.n,i+matB.ku+1); j++){
      newmat(i,j)+=matB(i,j);
    }
  }
  
  matA.destroy();
  matB.destroy();
  return _(newmat);
}
*/
//=============================================================================
/*! _dgsmatrix-_dgbmatrix operator */
/*
inline _dgematrix operator-(const _dgsmatrix& matA, const _dgbmatrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.n || matA.m!=matB.m){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a summation." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") + (" << matB.m << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  dgematrix newmat(matA);
  
  for(long i=0; i<matB.m; i++){
    for(long j=max(0,i-matB.kl); j<min(matB.n,i+matB.ku+1); j++){
      newmat(i,j)-=matB(i,j);
    }
  }
  
  matA.destroy();
  matB.destroy();
  return _(newmat);
}
*/
//=============================================================================
/*! _dgsmatrix*_dgbmatrix operator */
/*
inline _dgematrix operator*(const _dgsmatrix& matA, const _dgbmatrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.m){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a product." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") * (" << matB.m << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  dgematrix newmat( matA.m, matB.n );
  newmat.zero();
  
  for(long c=0; c<matA.vol; c++){
    for(long k=max(0,matA.jndx[c]-matB.ku);
        k<min(matB.m,matA.jndx[c]+matB.kl+1); k++){
      newmat(matA.indx[c],k) += matA.array[c]*matB(matA.jndx[c],k);
    }
  }
  
  matA.destroy();
  matB.destroy();
  return _(newmat);
}
*/
