//=============================================================================
/*! _drovector*_dsymatrix operator */
inline _drovector operator*(const _drovector& vec, const _dsymatrix& mat)
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
  
  dsymv_( 'l', mat.n, 1.0, mat.array, mat.n, vec.array, 1, 0.0, newvec.array, 1 );
  
  vec.destroy();
  mat.destroy();
  return _(newvec);
}
