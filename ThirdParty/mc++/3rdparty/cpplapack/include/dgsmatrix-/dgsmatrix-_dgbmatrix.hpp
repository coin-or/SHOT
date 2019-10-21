//=============================================================================
/*! dgsmatrix+_dgbmatrix operator */
inline _dgematrix operator+(const dgsmatrix& matA, const _dgbmatrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.n || matA.m!=matB.m){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a summation." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") + (" << matB.m << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  dgematrix newmat( matA.to_dgematrix() );
  
  for(long i=0; i<matB.m; i++){
    for(long j=std::max(long(0),i-matB.kl); j<std::min(matB.n,i+matB.ku+1); j++){
      newmat(i,j)+=matB(i,j);
    }
  }
  
  matB.destroy();
  return _(newmat);
}

//=============================================================================
/*! dgsmatrix-_dgbmatrix operator */
inline _dgematrix operator-(const dgsmatrix& matA, const _dgbmatrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.n || matA.m!=matB.m){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a summation." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") + (" << matB.m << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  dgematrix newmat( matA.to_dgematrix() );
  
  for(long i=0; i<matB.m; i++){
    for(long j=std::max(long(0),i-matB.kl); j<std::min(matB.n,i+matB.ku+1); j++){
      newmat(i,j)-=matB(i,j);
    }
  }
  
  matB.destroy();
  return _(newmat);
}

//=============================================================================
/*! dgsmatrix*_dgbmatrix operator */
inline _dgematrix operator*(const dgsmatrix& matA, const _dgbmatrix& matB)
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
  newmat.zero();
  
  for(std::vector<dcomponent>::const_iterator it=matA.data.begin(); it!=matA.data.end(); it++){
    for(long j=std::max(long(0),long(it->j)-matB.kl); j<std::min(matB.n,long(it->j)+matB.ku+1); j++){
      newmat(it->i,j) += it->v*matB(it->j,j);
    }
  }
  
  matB.destroy();
  return _(newmat);
}
