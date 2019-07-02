//=============================================================================
/*! _zhematrix*_zcovector operator */
inline _zcovector operator*(const _zhematrix& mat, const _zcovector& vec)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(mat.n!=vec.l){
    ERROR_REPORT;
    std::cerr << "These matrix and vector can not make a product." << std::endl
              << "Your input was (" << mat.n << "x" << mat.n << ") * (" << vec.l << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  zcovector newvec(mat.n);
  zhemv_( 'l', mat.n, comple(1.0,0.0), mat.array, mat.n,
          vec.array, 1, comple(0.0,0.0), newvec.array, 1 );
  
  mat.destroy();
  vec.destroy();
  return _(newvec);
}
