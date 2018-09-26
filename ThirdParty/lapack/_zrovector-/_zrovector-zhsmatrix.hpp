//=============================================================================
/*! _zrovector*zhsmatrix operator */
inline _zrovector operator*(const _zrovector& vec, const zhsmatrix& mat)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(vec.l!=mat.m){
    ERROR_REPORT;
    std::cerr << "These vector and matrix can not make a product." << std::endl
              << "Your input was (" << vec.l << ") * (" << mat.m << "x" << mat.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  zrovector newvec(mat.n);
  newvec.zero();
  
  for(std::vector<zcomponent>::const_iterator it=mat.data.begin(); it!=mat.data.end(); it++){
    newvec(it->j) +=vec(it->i)*it->v;
    if(it->i!=it->j){
      newvec(it->i) +=vec(it->j)*std::conj(it->v);
    }
  }
  
  vec.destroy();
  return _(newvec);
}
