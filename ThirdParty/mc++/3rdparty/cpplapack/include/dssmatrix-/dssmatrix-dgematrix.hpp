//=============================================================================
/*! dssmatrix+dgematrix operator */
/*
inline _dgematrix operator+(const dssmatrix& matA, const dgematrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.m!=matB.m || matA.n!=matB.n){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a summation." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") + (" << matB.m << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  dgematrix newmat(matB);
  for(long c=0; c<matA.vol; c++){
    newmat(matA.indx[c],matA.jndx[c]) += matA.array[c];
  }
  
  return _(newmat);
}
*/

//=============================================================================
/*! dssmatrix-dgematrix operator */
/*
inline _dgematrix operator-(const dssmatrix& matA, const dgematrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.m!=matB.m || matA.n!=matB.n){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a subtraction." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") - (" << matB.n << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG

  dgematrix newmat(-matB);
  for(long c=0; c<matA.vol; c++){
    newmat(matA.indx[c],matA.jndx[c]) += matA.array[c];
  }
  
  return _(newmat);
}
*/

//=============================================================================
/*! dssmatrix*dgematrix operator */
 /*
inline _dgematrix operator*(const dssmatrix& matA, const dgematrix& matB)
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
  
  double *ap(matA.array);
  long *ip(matA.indx), *kp(matA.jndx), c(0);
  while(c<matA.vol){
    for(long j=0; j<matB.n; j++){
      newmat(*ip,j) +=(*ap)*matB(*kp,j);
    }
    if((*ip)!=(*kp)){
      for(long j=0; j<matB.n; j++){
        newmat(*kp,j) +=(*ap)*matB(*ip,j);
      }
    }
    ap++; ip++; kp++; c++;
  }
  
  return _(newmat);
}
 */
