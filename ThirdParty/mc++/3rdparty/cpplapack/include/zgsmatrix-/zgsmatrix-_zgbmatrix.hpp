//=============================================================================
/*! zgsmatrix+_zgbmatrix operator */
inline _zgematrix operator+(const zgsmatrix& matA, const _zgbmatrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.n || matA.m!=matB.m){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a summation." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") + (" << matB.m << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  zgematrix newmat( matA.to_zgematrix() );
  
  for(long i=0; i<matB.m; i++){
    for(long j=std::max(0l,i-matB.kl); j<std::min(matB.n,i+matB.ku+1); j++){
      newmat(i,j) +=matB(i,j);
    }
  }
  
  matB.destroy();
  return _(newmat);
}

//=============================================================================
/*! zgsmatrix-_zgbmatrix operator */
inline _zgematrix operator-(const zgsmatrix& matA, const _zgbmatrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.n || matA.m!=matB.m){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a summation." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") + (" << matB.m << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  zgematrix newmat( matA.to_zgematrix() );
  
  for(long i=0; i<matB.m; i++){
    for(long j=std::max(0l,i-matB.kl); j<std::min(matB.n,i+matB.ku+1); j++){
      newmat(i,j) -=matB(i,j);
    }
  }
  
  matB.destroy();
  return _(newmat);
}

//=============================================================================
/*! zgsmatrix*_zgbmatrix operator */
inline _zgematrix operator*(const zgsmatrix& matA, const _zgbmatrix& matB)
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
  newmat.zero();
  
  for(std::vector<zcomponent>::const_iterator it=matA.data.begin(); it!=matA.data.end(); it++){
    for(long j=std::max(0l,long(it->j)-matB.kl); j<std::min(matB.n,long(it->j)+matB.ku+1); j++){
      newmat(it->i,j) += it->v*matB(it->j,j);
    }
  }
  
  matB.destroy();
  return _(newmat);
}
