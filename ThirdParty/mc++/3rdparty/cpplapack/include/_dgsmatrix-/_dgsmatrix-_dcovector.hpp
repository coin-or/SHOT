//=============================================================================
/*! _dgsmatrix*_dcovector operator */
inline _dcovector operator*(const _dgsmatrix& mat, const _dcovector& vec)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(mat.n!=vec.l){
    ERROR_REPORT;
    std::cerr << "These matrix and vector can not make a product." << std::endl
              << "Your input was (" << mat.m << "x" << mat.n << ") * (" << vec.l << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  dcovector newvec(mat.m);
  newvec.zero();
  
  for(std::vector<dcomponent>::const_iterator it=mat.data.begin(); it!=mat.data.end(); it++){
    newvec(it->i) += it->v * vec(it->j);
  }
  
  mat.destroy();
  vec.destroy();
  return _(newvec);
}
