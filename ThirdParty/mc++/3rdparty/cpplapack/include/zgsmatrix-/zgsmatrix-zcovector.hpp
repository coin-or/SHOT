//=============================================================================
/*! zgsmatrix*zcovector operator */
inline _zcovector operator*(const zgsmatrix& mat, const zcovector& vec)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(mat.n!=vec.l){
    ERROR_REPORT;
    std::cerr << "These matrix and vector can not make a product." << std::endl
              << "Your input was (" << mat.m << "x" << mat.n << ") * (" << vec.l << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  zcovector newvec(mat.m);
  newvec.zero();
  
  for(std::vector<zcomponent>::const_iterator it=mat.data.begin(); it!=mat.data.end(); it++){
    newvec(it->i) += it->v*vec(it->j);
  }
  
  return _(newvec);
}
