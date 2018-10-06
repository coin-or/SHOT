//=============================================================================
/*! _zhsmatrix+zhsmatrix operator */
inline _zhsmatrix operator+(const _zhsmatrix& matA, const zhsmatrix& matB)
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
  
  return _(newmat);
}

//=============================================================================
/*! _zhsmatrix-zhsmatrix operator */
inline _zhsmatrix operator-(const _zhsmatrix& matA, const zhsmatrix& matB)
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
  
  return _(newmat);
}

//=============================================================================
/*! _zhsmatrix*zhsmatrix operator */
/*
inline _zhsmatrix operator*(const _zhsmatrix& matA, const zhsmatrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.m){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a product." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") * (" << matB.m << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  zhsmatrix newmat( matA.m, matB.n, 0 );
  
  for(long c=0; c<matA.vol; c++){
    long k(matA.jndx[c]);
    std::vector< std::pair<long,long> >::iterator p;
    for(p=matB.Col[k].begin(); p!=matB.Col[k].end(); p++){
      newmat(matA.indx[c],p->first) +=matA.array[c]*matB.array[p->second];
    }
  }
  
  matA.destroy();
  return _(newmat);
}
*/
