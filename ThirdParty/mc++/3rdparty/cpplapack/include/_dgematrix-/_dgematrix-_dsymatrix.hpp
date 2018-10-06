//=============================================================================
/*! _dgematrix+_dsymatrix operator */
inline _dgematrix operator+(const _dgematrix& matA, const _dsymatrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.n || matA.m!=matB.n){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a summation." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") + (" << matB.n << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG

  for(long i=0; i<matB.n; i++) {
    for(long j=0; j<matB.n; j++) {
      matA(i,j)+=matB(i,j);
    }
  }
  
  matB.destroy();
  return matA;
}

//=============================================================================
/*! _dgematrix-_dsymatrix operator */
inline _dgematrix operator-(const _dgematrix& matA, const _dsymatrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.n || matA.m!=matB.n){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a summation." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") + (" << matB.n << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  for(long i=0; i<matB.n; i++) {
    for(long j=0; j<matB.n; j++) {
      matA(i,j)-=matB(i,j);
    }
  }
  
  matB.destroy();
  return matA;
}

//=============================================================================
/*! _dgematrix*_dsymatrix operator */
inline _dgematrix operator*(const _dgematrix& matA, const _dsymatrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.n){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a product." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") * (" << matB.n << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  dgematrix newmat( matA.m, matB.n );
  dsymm_( 'R', 'l', newmat.m, newmat.n, 1.0, matB.array, matB.n, 
          matA.array, matA.m, 0.0, newmat.array, newmat.m );
  
  matA.destroy();
  matB.destroy();
  return _(newmat);
}
