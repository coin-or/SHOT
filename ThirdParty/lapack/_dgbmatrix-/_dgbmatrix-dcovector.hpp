//=============================================================================
/*! _dgbmatrix*dcovector operator */
inline _dcovector operator*(const _dgbmatrix& mat, const dcovector& vec)
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
  dgbmv_( 'n', mat.m, mat.n, mat.kl, mat.ku, 1.0, mat.array,
          mat.kl+mat.ku+1, vec.array, 1, 0.0, newvec.array, 1 );
  
  mat.destroy();
  return _(newvec);
}
