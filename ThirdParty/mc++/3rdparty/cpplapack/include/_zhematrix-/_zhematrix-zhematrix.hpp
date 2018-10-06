//=============================================================================
/*! _zhematrix+zhematrix operator */
inline _zhematrix operator+(const _zhematrix& matA, const zhematrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.n){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a summation." << std::endl
              << "Your input was (" << matA.n << "x" << matA.n << ") + (" << matB.n << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  for(long i=0; i<matA.n; i++){
    for(long j=0; j<matA.n; j++){
      matA.array[i+matA.n*j]+=matB.array[i+matA.n*j];
    }
  }
  
  return matA;
}

//=============================================================================
/*! _zhematrix-zhematrix operator */
inline _zhematrix operator-(const _zhematrix& matA, const zhematrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.n){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a subtraction." << std::endl
              << "Your input was (" << matA.n << "x" << matA.n << ") - (" << matB.n << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  for(long i=0; i<matA.n; i++){
    for(long j=0; j<matA.n; j++){
      matA.array[i+matA.n*j]-=matB.array[i+matA.n*j];
    }
  }
  
  return matA;
}

//=============================================================================
/*! _zhematrix*zhematrix operator */
inline _zgematrix operator*(const _zhematrix& matA, const zhematrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.n){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a product." << std::endl
              << "Your input was (" << matA.n << "x" << matA.n << ") * (" << matB.n << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  matB.complete();
  zgematrix newmat( matA.n, matB.n );
  
  zhemm_( 'l', 'l', matA.n, matB.n, comple(1.0,0.0), 
          matA.array, matA.n, matB.array, matB.n, 
          comple(0.0,0.0), newmat.array, newmat.m );
  
  matA.destroy();
  return _(newmat);
}
