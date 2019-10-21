//=============================================================================
/*! zhsmatrix+_zhematrix operator */
/*
inline _zgematrix operator+(const zhsmatrix& matA, const _zhematrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.m!=matB.n || matA.n!=matB.n){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a summation." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") + (" << matB.n << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  zgematrix newmat(matB);
  for(long c=0; c<matA.vol; c++){
    newmat(matA.indx[c],matA.jndx[c]) += matA.array[c];
  }
  
  matB.destroy();
  return _(newmat);
}
*/

//=============================================================================
/*! zhsmatrix-_zhematrix operator */
/*
inline _zgematrix operator-(const zhsmatrix& matA, const _zhematrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.m!=matB.n || matA.n!=matB.n){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a subtraction." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") - (" << matB.n << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  zgematrix newmat(-matB);
  
  for(long c=0; c<matA.vol; c++){
    newmat(matA.indx[c],matA.jndx[c]) += matA.array[c];
  }
  
  matB.destroy();
  return _(newmat);
}
*/

//=============================================================================
/*! zhsmatrix*_zhematrix operator */
/*
inline _zgematrix operator*(const zhsmatrix& matA, const _zhematrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.n){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a product." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") * (" << matB.n << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  zgematrix newmat(matA.m, matB.n);
  newmat.zero();
  
  for(long c=0; c<matA.vol; c++){
    for(long i=0; i<matB.n; i++){
      newmat(matA.indx[c],i) += matA.array[c]*matB(matA.jndx[c],i);
    }
  }
  
  matB.destroy();
  return _(newmat);
}
*/
