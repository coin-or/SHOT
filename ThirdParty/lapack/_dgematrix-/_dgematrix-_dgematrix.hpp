//=============================================================================
/*! _dgematrix+_dgematrix operator */
inline _dgematrix operator+(const _dgematrix& matA, const _dgematrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.n || matA.m!=matB.m){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a summation." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") + (" << matB.m << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  for(long i=0; i<matA.m*matA.n; i++){ matA.array[i]+=matB.array[i]; }

  matB.destroy();
  return matA;
}

//=============================================================================
/*! _dgematrix-_dgematrix operator */
inline _dgematrix operator-(const _dgematrix& matA, const _dgematrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.n || matA.m!=matB.m){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a subtraction." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") - (" << matB.m << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG

  for(long i=0; i<matA.m*matA.n; i++){ matA.array[i]-=matB.array[i]; }
  
  matB.destroy();
  return matA;
}

//=============================================================================
/*! _dgematrix*_dgematrix operator */
inline _dgematrix operator*(const _dgematrix& matA, const _dgematrix& matB)
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
  dgemm_( 'n', 'n', matA.m, matB.n, matA.n, 1.0, matA.array, matA.m,
          matB.array, matB.m, 0.0, newmat.array, matA.m );
  
  matA.destroy();
  matB.destroy();
  return _(newmat);
}
