//=============================================================================
/*! dsymatrix+_dgsmatrix operator */
inline _dgematrix operator+(const dsymatrix& matA, const _dgsmatrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.m || matA.n!=matB.n){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a summation." << std::endl
              << "Your input was (" << matA.n << "x" << matA.n << ") + (" << matB.m << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  dgematrix newmat( matA.to_dgematrix() );
  for(std::vector<dcomponent>::const_iterator it=matB.data.begin(); it!=matB.data.end(); it++){
    newmat(it->i,it->j) += it->v;
  }
  
  matB.destroy();
  return _(newmat);
}

//=============================================================================
/*! dsymatrix-_dgsmatrix operator */
inline _dgematrix operator-(const dsymatrix& matA, const _dgsmatrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.m || matA.n!=matB.n){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a subtraction." << std::endl
              << "Your input was (" << matA.n << "x" << matA.n << ") - (" << matB.m << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG

  dgematrix newmat( matA.to_dgematrix() );
  for(std::vector<dcomponent>::const_iterator it=matB.data.begin(); it!=matB.data.end(); it++){
    newmat(it->i,it->j) -= it->v;
  }
  
  matB.destroy();
  return _(newmat);
}

//=============================================================================
/*! dsymatrix*_dgsmatrix operator */
inline _dgematrix operator*(const dsymatrix& matA, const _dgsmatrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.m){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a product." << std::endl
              << "Your input was (" << matA.n << "x" << matA.n << ") * (" << matB.m << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  dgematrix newmat(matA.n, matB.n);
  newmat.zero();
  for(std::vector<dcomponent>::const_iterator it=matB.data.begin(); it!=matB.data.end(); it++){
    for(long i=0; i<matA.n; i++){
      newmat(i,it->j) += matB(i,it->i)*it->v;
    }
  }
  
  matB.destroy();
  return _(newmat);
}
