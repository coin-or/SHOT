//=============================================================================
/*! dssmatrix+_dgbmatrix operator */
/*
inline _dgematrix operator+(const dssmatrix& matA, const _dgbmatrix& matB)
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
  
  matB.destroy();
  return _(newmat);
}
*/
//=============================================================================
/*! dssmatrix-_dgbmatrix operator */
/*
inline _dgematrix operator-(const dssmatrix& matA, const _dgbmatrix& matB)
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
  
  matB.destroy();
  return _(newmat);
}
*/

//=============================================================================
/*! dssmatrix*_dgbmatrix operator */
/*
inline _dgematrix operator*(const dssmatrix& matA, const _dgbmatrix& matB)
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
    for(long j=max(0,matA.jndx[c]-matB.kl);
        j<min(matB.n,matA.jndx[c]+matB.ku+1); j++){
      newmat(matA.indx[c],j) += matA.array[c]*matB(matA.jndx[c],j);
    }
  }
  
  matB.destroy();
  return _(newmat);
}
*/
