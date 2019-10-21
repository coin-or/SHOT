//=============================================================================
/*! zgematrix+zhematrix operator */
inline _zgematrix operator+(const zgematrix& matA, const zhematrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.m!=matB.m || matA.n!=matB.n){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a summation." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") + (" << matB.m << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  zgematrix newmat(matA);
  for(long c=0; c<matB.vol; c++){
    newmat(matB.indx[c],matB.jndx[c]) += matB.array[c];
  }
  
  return _(newmat);
}

//=============================================================================
/*! zgematrix-zhematrix operator */
inline _zgematrix operator-(const zgematrix& matA, const zhematrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.m!=matB.m || matA.n!=matB.n){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a subtraction." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") - (" << matB.n << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG

  zgematrix newmat(matA);
  for(long c=0; c<matB.vol; c++){
    newmat(matB.indx[c],matB.jndx[c]) -= matB.array[c];
  }
  
  return _(newmat);
}

//=============================================================================
/*! zgematrix*zhematrix operator */
inline _zgematrix operator*(const zgematrix& matA, const zhematrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.m){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a product." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") * (" << matB.n << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  zgematrix newmat(matA.m, matB.n);
  newmat.zero();
  
  for(long c=0; c<matB.vol; c++){
    for(long i=0; i<matA.m; i++){
      newmat(i,matB.jndx[c]) += matA(i,matB.indx[c])*matB.array[c];
    }
  }
  
  return _(newmat);
}
