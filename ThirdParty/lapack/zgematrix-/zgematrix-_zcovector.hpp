//=============================================================================
/*! zgematrix*_zcovector operator */
inline _zcovector operator*(const zgematrix& mat, const _zcovector& vec)
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
  zgemv_( 'n', mat.m, mat.n, comple(1.0,0.0), mat.array, mat.m,
          vec.array, 1, comple(0.0,0.0), newvec.array, 1 );
  
  vec.destroy();
  return _(newvec);
}
