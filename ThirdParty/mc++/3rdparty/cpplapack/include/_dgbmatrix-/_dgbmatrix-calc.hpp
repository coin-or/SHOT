//=============================================================================
/*! return its transposed dgbmatrix */
inline _dgbmatrix t(const _dgbmatrix& mat)
{VERBOSE_REPORT;
  dgbmatrix newmat(mat.n, mat.m, mat.ku, mat.kl);
  
  for(long i=0; i<newmat.m; i++){
    for(long j=std::max(long(0),i-newmat.kl); j<std::min(newmat.n,i+newmat.ku+1); j++){
      newmat(i,j) =mat(j,i);
    }
  }
  
  mat.destroy();
  return _(newmat);
}

//=============================================================================
/*! return its inverse matrix */
inline _dgematrix i(const _dgbmatrix& mat)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(mat.m!=mat.n){
    ERROR_REPORT;
    std::cerr << "This matrix is not square and has no inverse matrix." << std::endl
              << "Your input was (" << mat.m << "x" << mat.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  dgbmatrix mat_cp(mat);
  dgematrix mat_inv(mat_cp.m,mat_cp.n);
  mat_inv.identity();
  mat_cp.dgbsv(mat_inv);

  return _(mat_inv);
}
