//=============================================================================
/*! drovector*dssmatrix operator */
inline _drovector operator*(const drovector& vec, const dssmatrix& mat)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(vec.l!=mat.n){
    ERROR_REPORT;
    std::cerr << "These vector and matrix can not make a product." << std::endl
              << "Your input was (" << vec.l << ") * (" << mat.n << "x" << mat.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  drovector newvec(mat.n);
  newvec.zero();  
  for(std::vector<dcomponent>::const_iterator it=mat.data.begin(); it!=mat.data.end(); it++){
    newvec(it->j) += vec(it->i)*it->v;
    if(it->i!=it->j){
      newvec(it->i) +=vec(it->j)*it->v;
    }
  }
  
  return _(newvec);
}
