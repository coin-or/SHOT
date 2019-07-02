//=============================================================================
/*! dgematrix*_dcovector operator */
inline _dcovector operator*(const dgematrix& mat, const _dcovector& vec)
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
  dgemv_( 'n', mat.m, mat.n, 1.0, mat.array, mat.m,
          vec.array, 1, 0.0, newvec.array, 1 );
  
  vec.destroy();
  return _(newvec);
}
