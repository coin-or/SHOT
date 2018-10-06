//=============================================================================
/*! dgsmatrix+_dsymatrix operator */
inline _dgematrix operator+(const dgsmatrix& matA, const _dsymatrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.m!=matB.m || matA.n!=matB.n){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a summation." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") + (" << matB.m << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  dgematrix newmat( matB.to_dgematrix() );
  for(std::vector<dcomponent>::const_iterator it=matA.data.begin(); it!=matA.data.end(); it++){
    newmat(it->i,it->j) += it->v;
  }
  matB.destroy();
  return _(newmat);
}

//=============================================================================
/*! dgsmatrix-_dsymatrix operator */
inline _dgematrix operator-(const dgsmatrix& matA, const _dsymatrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.m!=matB.m || matA.n!=matB.n){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a subtraction." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") - (" << matB.m << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  dgematrix newmat( (-matB).to_dgematrix() );
  for(std::vector<dcomponent>::const_iterator it=matA.data.begin(); it!=matA.data.end(); it++){
    newmat(it->i,it->j) += it->v;
  }
  matB.destroy();
  return _(newmat);
}

//=============================================================================
/*! dgsmatrix*_dsymatrix operator */
inline _dgematrix operator*(const dgsmatrix& matA, const _dsymatrix& matB)
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
    for(long i=0; i<matB.n; i++){
      newmat(it->i,i) += it->v*matB(it->j,i);
    }
  }
  
  matB.destroy();
  return _(newmat);
}
