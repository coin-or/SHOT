//=============================================================================
/*! _dsymatrix+dgematrix operator */
inline _dgematrix operator+(const _dsymatrix& matA, const dgematrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.n || matA.n!=matB.m){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a summation." << std::endl
              << "Your input was (" << matA.n << "x" << matA.n << ") + (" << matB.m << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  dgematrix newmat(matB);
  for(long i=0; i<matA.n; i++){
    for(long j=0; j<matA.n; j++ ){
      newmat(i,j)+=matA(i,j);
    }
  }
  
  matA.destroy();
  return _(newmat);
}

//=============================================================================
/*! _dsymatrix-dgematrix operator */
inline _dgematrix operator-(const _dsymatrix& matA, const dgematrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.n || matA.n!=matB.m){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a subtraction." << std::endl
              << "Your input was (" << matA.n << "x" << matA.n << ") - (" << matB.m << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  dgematrix newmat(-matB);
  for(long i=0; i<matA.n; i++){
    for(long j=0; j<matA.n; j++ ){
      newmat(i,j)+=matA(i,j);
    }
  }
  
  matA.destroy();
  return _(newmat);
}

//=============================================================================
/*! _dsymatrix*dgematrix operator */
inline _dgematrix operator*(const _dsymatrix& matA, const dgematrix& matB)
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
  dsymm_( 'l', 'l', matA.n, matB.n, 1.0, matA.array, matA.n, 
          matB.array, matB.m, 0.0, newmat.array, newmat.m );
  
  matA.destroy();
  return _(newmat);
}
