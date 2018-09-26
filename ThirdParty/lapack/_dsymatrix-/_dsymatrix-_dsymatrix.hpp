//=============================================================================
/*! _dsymatrix+_dsymatrix operator */
inline _dsymatrix operator+(const _dsymatrix& matA, const _dsymatrix& matB)
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
    for(long j=0; j<=i; j++){
      matA.darray[j][i] +=matB.darray[j][i];
    }
  }

  matB.destroy();
  return matA;
}

//=============================================================================
/*! _dsymatrix-_dsymatrix operator */
inline _dsymatrix operator-(const _dsymatrix& matA, const _dsymatrix& matB)
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
    for(long j=0; j<=i; j++){
      matA.darray[j][i] -=matB.darray[j][i];
    }
  }
  
  matB.destroy();
  return matA;
}

//=============================================================================
/*! _dsymatrix*_dsymatrix operator */
inline _dgematrix operator*(const _dsymatrix& matA, const _dsymatrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.n){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a product." << std::endl
              << "Your input was (" << matA.n << "x" << matA.n << ") * (" << matB.n << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  matA.complete();
  matB.complete();
  dgematrix newmat(matA.n, matA.n);
  
  dgemm_( 'n', 'n', matA.n, matB.n, matA.n, 1.0, matA.array, matA.n,
          matB.array, matB.n, 0.0, newmat.array, matA.n );
  
  matA.destroy();
  matB.destroy();
  return _(newmat);
}
