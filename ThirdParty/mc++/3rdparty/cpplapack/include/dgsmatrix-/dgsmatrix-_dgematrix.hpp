//=============================================================================
/*! dgsmatrix+_dgematrix operator */
inline _dgematrix operator+(const dgsmatrix& matA, const _dgematrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.m!=matB.m || matA.n!=matB.n){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a summation." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") + (" << matB.m << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  for(std::vector<dcomponent>::const_iterator it=matA.data.begin(); it!=matA.data.end(); it++){
    matB(it->i,it->j) += it->v;
  }
  return matB;
}

//=============================================================================
/*! dgsmatrix-_dgematrix operator */
inline _dgematrix operator-(const dgsmatrix& matA, const _dgematrix& matB)
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
  for(long i=0; i<matB.m*matB.n; i++){
    matB.array[i]=-matB.array[i];
  }
  
  //// add ////
  for(std::vector<dcomponent>::const_iterator it=matA.data.begin(); it!=matA.data.end(); it++){
    matB(it->i,it->j) += it->v;
  }
  
  return matB;
}

//=============================================================================
/*! dgsmatrix*_dgematrix operator */
inline _dgematrix operator*(const dgsmatrix& matA, const _dgematrix& matB)
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
  
  for(std::vector<dcomponent>::const_iterator it=matA.data.begin(); it!=matA.data.end(); it++){
    for(long j=0; j<matB.n; j++){
      newmat(it->i,j) += it->v *matB(it->j,j);
    }
  }
  
  matB.destroy();
  return _(newmat);
}
