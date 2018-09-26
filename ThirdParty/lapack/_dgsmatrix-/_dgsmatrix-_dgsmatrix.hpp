//=============================================================================
/*! _dgsmatrix+_dgsmatrix operator */
inline _dgsmatrix operator+(const _dgsmatrix& matA, const _dgsmatrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.n || matA.m!=matB.m){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a summation." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") + (" << matB.m << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  dgsmatrix newmat(matA);
  for(std::vector<dcomponent>::const_iterator it=matB.data.begin(); it!=matB.data.end(); it++){
    newmat(it->i,it->j) += it->v;
  }
  
  matB.destroy();
  return _(newmat);
}

//=============================================================================
/*! _dgsmatrix-_dgsmatrix operator */
inline _dgsmatrix operator-(const _dgsmatrix& matA, const _dgsmatrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.n || matA.m!=matB.m){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a subtraction." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") - (" << matB.m << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  dgsmatrix newmat(matA);
  for(std::vector<dcomponent>::const_iterator it=matB.data.begin(); it!=matB.data.end(); it++){
    newmat(it->i,it->j) -= it->v;
  }
  
  matB.destroy();
  return _(newmat);
}

//=============================================================================
/*! _dgsmatrix*_dgsmatrix operator */
inline _dgsmatrix operator*(const _dgsmatrix& matA, const _dgsmatrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.m){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a product." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") * (" << matB.m << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  dgsmatrix newmat(matA.m, matB.n);
  
  for(std::vector<dcomponent>::const_iterator it=matA.data.begin(); it!=matA.data.end(); it++){
    long k(it->j);
    std::vector<uint32_t>::iterator p;
    for(p=matB.rows[k].begin(); p!=matB.rows[k].end(); p++){
      newmat(it->i,matB.data[*p].j) += it->v*matB.data[*p].v;
    }
  }
  
  matA.destroy();
  matB.destroy();
  return _(newmat);
}
