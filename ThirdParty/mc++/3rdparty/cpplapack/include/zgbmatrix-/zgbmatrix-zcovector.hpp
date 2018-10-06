//=============================================================================
/*! zgbmatrix*zcovector operator */
inline _zcovector operator*(const zgbmatrix& mat, const zcovector& vec)
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
  zgbmv_( 'n', mat.m, mat.n, mat.kl, mat.ku, comple(1.0,0.0), mat.array,
          mat.kl+mat.ku+1, vec.array, 1, comple(0.0,0.0), newvec.array, 1 );
  
  return _(newvec);
}
