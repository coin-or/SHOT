//=============================================================================
/*! _dgbmatrix+dgbmatrix operator */
inline _dgbmatrix operator+(const _dgbmatrix& matA, const dgbmatrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.n || matA.m!=matB.m){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a summation." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") + (" << matB.m << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  if(matA.kl>=matB.kl && matA.ku>=matB.ku){
    for(long i=0; i<matB.m; i++){
      for(long j=std::max(long(0),i-matB.kl); j<std::min(matB.n,i+matB.ku+1); j++){
        matA(i,j)+=matB(i,j);
      }
    }
    
    return matA;
  }
  
  else{
    dgbmatrix newmat(matA.m,matA.n,std::max(matA.kl,matB.kl),std::max(matA.ku,matB.ku));
    newmat.zero();
    
    for(long i=0; i<matA.m; i++){
      for(long j=std::max(long(0),i-matA.kl); j<std::min(matA.n,i+matA.ku+1); j++){
        newmat(i,j)+=matA(i,j);
      }
      for(long j=std::max(long(0),i-matB.kl); j<std::min(matB.n,i+matB.ku+1); j++){
        newmat(i,j)+=matB(i,j);
      }
    }
    
    matA.destroy();
    return _(newmat);
  }
}

//=============================================================================
/*! _dgbmatrix-dgbmatrix operator */
inline _dgbmatrix operator-(const _dgbmatrix& matA, const dgbmatrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.n || matA.m!=matB.m){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a subtraction." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") - (" << matB.m << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  if(matA.kl>=matB.kl && matA.ku>=matB.ku){
    for(long i=0; i<matB.m; i++){
      for(long j=std::max(long(0),i-matB.kl); j<std::min(matB.n,i+matB.ku+1); j++){
        matA(i,j)-=matB(i,j);
      }
    }
    
    return matA;
  }
  
  else{
    dgbmatrix newmat(matA.m,matA.n,std::max(matA.kl,matB.kl),std::max(matA.ku,matB.ku));
    newmat.zero();
    
    for(long i=0; i<matA.m; i++){
      for(long j=std::max(long(0),i-matA.kl); j<std::min(matA.n,i+matA.ku+1); j++){
        newmat(i,j)+=matA(i,j);
      }
      for(long j=std::max(long(0),i-matB.kl); j<std::min(matB.n,i+matB.ku+1); j++){
        newmat(i,j)-=matB(i,j);
      }
    }
    
    return _(newmat);
  }
}

//=============================================================================
/*! _dgbmatrix*dgbmatrix operator */
inline _dgbmatrix operator*(const _dgbmatrix& matA, const dgbmatrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.m){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a product." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") * (" << matB.m << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  dgbmatrix newmat( matA.m, matB.n, std::min(matA.kl+matB.kl,matA.m-1), std::min(matA.ku+matB.ku,matB.n-1) );
  newmat.zero();
  
  for(long i=0; i<newmat.m; i++){
    for(long j=std::max(long(0),i-newmat.kl); j<std::min(newmat.n,i+newmat.ku+1); j++){
      for(long k=std::max( std::max(long(0),i-matA.kl), std::max(long(0),j-matB.ku) );
          k< std::min( std::min(matA.n,i+matA.ku+1), std::min(matB.m,j+matB.kl+1) ); k++){
        newmat(i,j)+= matA(i,k)*matB(k,j);
      }
    }
  }
  
  matA.destroy();
  return _(newmat);
}
