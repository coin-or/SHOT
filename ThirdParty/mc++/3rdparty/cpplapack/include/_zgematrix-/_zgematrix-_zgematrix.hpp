//=============================================================================
/*! _zgematrix+_zgematrix operator */
inline _zgematrix operator+(const _zgematrix& matA, const _zgematrix& matB)
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
/*! _zgematrix-_zgematrix operator */
inline _zgematrix operator-(const _zgematrix& matA, const _zgematrix& matB)
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
/*! _zgematrix*_zgematrix operator */
inline _zgematrix operator*(const _zgematrix& matA, const _zgematrix& matB)
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
  zgemm_( 'n', 'n', matA.m, matB.n, matA.n, comple(1.0,0.0),
          matA.array, matA.m, matB.array, matB.m, 
          comple(0.0,0.0), newmat.array, matA.m );
  
  matA.destroy();
  matB.destroy();
  return _(newmat);
}
