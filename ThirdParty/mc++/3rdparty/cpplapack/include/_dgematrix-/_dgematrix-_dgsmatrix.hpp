//=============================================================================
/*! _dgematrix+_dgsmatrix operator */
inline _dgematrix operator+(const _dgematrix& matA, const _dgsmatrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.m!=matB.m || matA.n!=matB.n){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a summation." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") + (" << matB.m << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  for(std::vector<dcomponent>::const_iterator it=matB.data.begin(); it!=matB.data.end(); it++){
    matA(it->i,it->j) += it->v;
  }
  
  matB.destroy();
  return matA;
}

//=============================================================================
/*! _dgematrix-_dgsmatrix operator */
inline _dgematrix operator-(const _dgematrix& matA, const _dgsmatrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.m!=matB.m || matA.n!=matB.n){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a subtraction." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") - (" << matB.m << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  //// change sign ////
  for(long i=0; i<matA.m*matA.n; i++){
    matA.array[i]=-matA.array[i];
  }
  
  //// add ////
  for(std::vector<dcomponent>::const_iterator it=matB.data.begin(); it!=matB.data.end(); it++){
    matA(it->i,it->j) += it->v;
  }
  
  matB.destroy();
  return matA;
}

//=============================================================================
/*! _dgematrix*_dgsmatrix operator */
inline _dgematrix operator*(const _dgematrix& matA, const _dgsmatrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.m){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a product." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") * (" << matB.m << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  dgematrix newmat(matA.m, matB.n);
  newmat.zero();
  for(std::vector<dcomponent>::const_iterator it=matB.data.begin(); it!=matB.data.end(); it++){
    for(long i=0; i<matA.m; i++){
      newmat(i,it->j) += matA(i,it->i)*it->v;
    }
  }
  
  matA.destroy();
  matB.destroy();
  return _(newmat);
}
