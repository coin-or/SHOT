//=============================================================================
/*! dgbmatrix+dgsmatrix operator */
inline _dgematrix operator+(const dgbmatrix& matA, const dgsmatrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.n || matA.m!=matB.m){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a summation." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") + (" << matB.m << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  dgematrix newmat( matB.to_dgematrix() );
  
  for(long i=0; i<matA.m; i++){
    for(long j=std::max(long(0),i-matA.kl); j<std::min(matA.n,i+matA.ku+1); j++){
      newmat(i,j)+=matA(i,j);
    }
  }
  
  return _(newmat);
}

//=============================================================================
/*! dgbmatrix-dgsmatrix operator */
inline _dgematrix operator-(const dgbmatrix& matA, const dgsmatrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.n || matA.m!=matB.m){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a summation." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") + (" << matB.m << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  dgematrix newmat( -matB.to_dgematrix() );
  
  for(long i=0; i<matA.m; i++){
    for(long j=std::max(long(0),i-matA.kl); j<std::min(matA.n,i+matA.ku+1); j++){
      newmat(i,j)-=matA(i,j);
    }
  }
  
  return _(newmat);
}

//=============================================================================
/*! dgbmatrix*dgsmatrix operator */
inline _dgematrix operator*(const dgbmatrix& matA, const dgsmatrix& matB)
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
  for(std::vector<dcomponent>::const_iterator it=matB.data.begin(); it!=matB.data.end(); it++){
    for(long i=std::max(long(0),long(it->i)-(matA.ku+1)); i<std::min(matA.m,long(it->i)+matA.kl); i++){
      newmat(i,it->j) += matA(i,it->i)*it->v;
    }
  }
  
  return _(newmat);
}
