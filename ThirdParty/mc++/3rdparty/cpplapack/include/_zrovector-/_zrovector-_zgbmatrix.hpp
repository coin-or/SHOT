//=============================================================================
/*! _zrovector*_zgbmatrix operator */
inline _zrovector operator*(const _zrovector& vec, const _zgbmatrix& mat)
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
  zgbmv_( 'T', mat.m, mat.n, mat.kl, mat.ku, comple(1.0,0.0),
          mat.array, mat.kl+mat.ku+1, vec.array, 1, comple(0.0,0.0), newvec.array, 1 );
  
  vec.destroy();
  mat.destroy();
  return _(newvec);
}
