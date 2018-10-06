//=============================================================================
/*! dssmatrix+_dgematrix operator */
/*
inline _dgematrix operator+(const dssmatrix& matA, const _dgematrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.m!=matB.m || matA.n!=matB.n){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a summation." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") + (" << matB.m << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  for(long c=0; c<matA.vol; c++){
    matB(matA.indx[c],matA.jndx[c]) += matA.array[c];
    matB(matA.jndx[c],matA.indx[c]) += matA.array[c];
  }
  
  return matB;
}
*/

//=============================================================================
/*! dssmatrix-_dgematrix operator */
/*
inline _dgematrix operator-(const dssmatrix& matA, const _dgematrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.m!=matB.m || matA.n!=matB.n){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a subtraction." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") - (" << matB.n << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  //// change sign ////
  for(long i=0; i<matB.m*matB.n; i++){
    matB.array[i]=-matB.array[i];
  }
  
  //// add ////
  for(long c=0; c<matA.vol; c++){
    matB(matA.indx[c],matA.jndx[c]) += matA.array[c];
  }
  
  return matB;
}
*/
//=============================================================================
/*! dssmatrix*_dgematrix operator */
/*
inline _dgematrix operator*(const dssmatrix& matA, const _dgematrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.m){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a product." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") * (" << matB.n << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  dgematrix newmat(matA.m, matB.n);
  newmat.zero();
  
  for(long c=0; c<matA.vol; c++){
    for(long j=0; j<matB.n; j++){
      newmat(matA.indx[c],j) += matA.array[c]*matB(matA.jndx[c],j);
    }
  }
  
  matB.destroy();
  return _(newmat);
}
*/
