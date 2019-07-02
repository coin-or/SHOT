//=============================================================================
/*! dssmatrix*_dcovector operator */
inline _dcovector operator*(const dssmatrix& mat, const _dcovector& vec)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(mat.n!=vec.l){
    ERROR_REPORT;
    std::cerr << "These matrix and vector can not make a product." << std::endl
              << "Your input was (" << mat.n << "x" << mat.n << ") * (" << vec.l << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  dcovector newvec(dcovector(mat.n));
  newvec.zero();
  
  for(std::vector<dcomponent>::const_iterator it=mat.data.begin(); it!=mat.data.end(); it++){
    newvec(it->i) +=it->v*vec(it->j);
    if(it->i!=it->j){
      newvec(it->j) +=it->v*vec(it->i);
    }
  }
  
  vec.destroy();
  return _(newvec);
}
