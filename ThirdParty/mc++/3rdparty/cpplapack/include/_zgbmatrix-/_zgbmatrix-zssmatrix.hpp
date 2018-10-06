//=============================================================================
/*! _zgbmatrix+zhematrix operator */
inline _zgematrix operator+(const _zgbmatrix& matA, const zhematrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.n || matA.m!=matB.m){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a summation." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") + (" << matB.m << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  zgematrix newmat(matB);
  
  for(long i=0; i<matA.m; i++){
    for(long j=max(0,i-matA.kl); j<min(matA.n,i+matA.ku+1); j++){
      newmat(i,j)+=matA(i,j);
    }
  }
  
  matA.destroy();
  return _(newmat);
}

//=============================================================================
/*! _zgbmatrix-zhematrix operator */
inline _zgematrix operator-(const _zgbmatrix& matA, const zhematrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.n || matA.m!=matB.m){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a summation." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") + (" << matB.m << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  zgematrix newmat(-matB);
  
  for(long i=0; i<matA.m; i++){
    for(long j=max(0,i-matA.kl); j<min(matA.n,i+matA.ku+1); j++){
      newmat(i,j)-=matA(i,j);
    }
  }
  
  matA.destroy();
  return _(newmat);
}

//=============================================================================
/*! _zgbmatrix*zhematrix operator */
inline _zgematrix operator*(const _zgbmatrix& matA, const zhematrix& matB)
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
  
  for(long c=0; c<matB.vol; c++){
    for(long i=max(0,matB.indx[c]-(matA.ku+1));
        i<min(matA.m,matB.indx[c]+matA.kl); i++){
      newmat(i,matB.jndx[c]) += matA(i,matB.indx[c])*matB.array[c];
    }
  }
  
  matA.destroy();
  return _(newmat);
}
