//=============================================================================
/*! _dgematrix*dssmatrix operator */
inline _dgematrix operator*(const _dgematrix& matA, const dssmatrix& matB)
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
  
  for(std::vector<dcomponent>::const_iterator it=matB.data.begin(); it!=matB.data.end(); it++){
    for(long i=0; i<matA.m; i++){
      newmat(i,it->j) +=matA(i,it->i)*it->v;
    }
    if(it->i!=it->j){
      for(long i=0; i<matA.m; i++){
        newmat(i,it->i) +=matA(i,it->j)*it->v;
      }
    }
  }
  
  matA.destroy();
  return _(newmat);
}
