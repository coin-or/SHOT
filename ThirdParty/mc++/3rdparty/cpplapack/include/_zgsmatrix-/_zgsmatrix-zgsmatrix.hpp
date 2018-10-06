//=============================================================================
/*! _zgsmatrix+zgsmatrix operator */
inline _zgsmatrix operator+(const _zgsmatrix& matA, const zgsmatrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.n || matA.m!=matB.m){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a summation." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") + (" << matB.m << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG

  zgsmatrix newmat(matA);
  
  for(std::vector<zcomponent>::const_iterator it=matB.data.begin(); it!=matB.data.end(); it++){
    newmat(it->i, it->j) +=it->v;
  }
  
  return _(newmat);
}

//=============================================================================
/*! _zgsmatrix-zgsmatrix operator */
inline _zgsmatrix operator-(const _zgsmatrix& matA, const zgsmatrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.n || matA.m!=matB.m){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a subtraction." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") - (" << matB.m << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG

  zgsmatrix newmat(matA);
  
  for(std::vector<zcomponent>::const_iterator it=matB.data.begin(); it!=matB.data.end(); it++){
    newmat(it->i, it->j) -=it->v;
  }
  
  return _(newmat);
}

//=============================================================================
/*! _zgsmatrix*zgsmatrix operator */
inline _zgsmatrix operator*(const _zgsmatrix& matA, const zgsmatrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.m){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a product." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") * (" << matB.m << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  zgsmatrix newmat(matA.m, matB.n);
  
  for(std::vector<zcomponent>::const_iterator it=matA.data.begin(); it!=matA.data.end(); it++){
    long k(it->j);
    std::vector<uint32_t>::const_iterator p;
    for(p=matB.rows[k].begin(); p!=matB.rows[k].end(); p++){
      newmat(it->i,matB.data[*p].j) +=it->v*matB.data[*p].v;
    }
  }
  
  matA.destroy();
  return _(newmat);
}
