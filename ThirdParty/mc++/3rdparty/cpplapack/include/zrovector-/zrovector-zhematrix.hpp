//=============================================================================
/*! zrovector*zhematrix operator */
inline _zrovector operator*(const zrovector& vec, const zhematrix& mat)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(vec.l!=mat.n){
    ERROR_REPORT;
    std::cerr << "These vector and matrix can not make a product." << std::endl
              << "Your input was (" << vec.l << ") * (" << mat.n << "x" << mat.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  zrovector newvec(mat.n);
  zhemv_( 'l', mat.n, comple(1.0,0.0), mat.array, mat.n,
          vec.array, 1, comple(0.0,0.0), newvec.array, 1 );
  
  return _(newvec);
}
