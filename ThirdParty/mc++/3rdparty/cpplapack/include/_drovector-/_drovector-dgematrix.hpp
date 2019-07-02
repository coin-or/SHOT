//=============================================================================
/*! _drovector*dgematrix operator */
inline _drovector operator*(const _drovector& vec, const dgematrix& mat)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(vec.l!=mat.m){
    ERROR_REPORT;
    std::cerr << "These vector and matrix can not make a product." << std::endl
              << "Your input was (" << vec.l << ") * (" << mat.m << "x" << mat.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  drovector newvec(mat.n);
  
  dgemv_( 'T', mat.m, mat.n, 1.0, mat.array, mat.m, vec.array, 1, 0.0, newvec.array, 1 );
  
  vec.destroy();
  return _(newvec);
}
