//=============================================================================
/*! _zhsmatrix+_zhsmatrix operator */
inline _zhsmatrix operator+(const _zhsmatrix& matA, const _zhsmatrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.n){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a summation." << std::endl
              << "Your input was (" << matA.n << "x" << matA.n << ") + (" << matB.n << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  zhsmatrix newmat(matA);
  
  for(std::vector<zcomponent>::const_iterator it=matB.data.begin(); it!=matB.data.end(); it++){
    newmat(it->i,it->j) +=it->v;
  }
  
  matB.destroy();
  return _(newmat);
}

//=============================================================================
/*! _zhsmatrix-_zhsmatrix operator */
inline _zhsmatrix operator-(const _zhsmatrix& matA, const _zhsmatrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.n){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a subtraction." << std::endl
              << "Your input was (" << matA.n << "x" << matA.n << ") - (" << matB.n << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  zhsmatrix newmat(matA);
  
  for(std::vector<zcomponent>::const_iterator it=matB.data.begin(); it!=matB.data.end(); it++){
    newmat(it->i,it->j) -=it->v;
  }
  
  matB.destroy();
  return _(newmat);
}

//=============================================================================
/*! _zhsmatrix*_zhsmatrix operator */
/*
inline _zhsmatrix operator*(const _zhsmatrix& matA, const _zhsmatrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.n){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a product." << std::endl
              << "Your input was (" << matA.n << "x" << matA.n << ") * (" << matB.n << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  zhsmatrix newmat(matA.n);
  
  for(long c=0; c<matA.vol; c++){
    long k(matA.jndx[c]);
    std::vector< std::pair<long,long> >::iterator p;
    for(p=matB.line[k].begin(); p!=matB.line[k].end(); p++){
      newmat(matA.indx[c],p->first) +=matA.array[c]*matB.array[p->second];
    }
  }
  
  matA.destroy();
  matB.destroy();
  return _(newmat);
}
*/
