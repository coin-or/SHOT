//=============================================================================
/*! drovector*dgbmatrix operator */
inline _drovector operator*(const drovector& vec, const dgbmatrix& mat)
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
  dgbmv_( 'T', mat.m, mat.n, mat.kl, mat.ku, 1.0,
          mat.array, mat.kl+mat.ku+1, vec.array, 1, 0.0, newvec.array, 1 );
  
  return _(newvec);
}
