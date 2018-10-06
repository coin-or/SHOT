//=============================================================================
/*! return transposed zgbmatrix */
inline _zgbmatrix t(const zgbmatrix& mat)
{VERBOSE_REPORT;
  zgbmatrix newmat(mat.n, mat.m, mat.ku, mat.kl);
  for(long i=0; i<newmat.m; i++){
    for(long j=std::max(long(0),i-newmat.kl); j<std::min(newmat.n,i+newmat.ku+1); j++){
      newmat(i,j) =mat(j,i);
    }
  }
  
  return _(newmat);
}

//=============================================================================
/*! return its inverse matrix */
inline _zgematrix i(const zgbmatrix& mat)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(mat.m!=mat.n){
    ERROR_REPORT;
    std::cerr << "This matrix is not square and has no inverse matrix." << std::endl
              << "Your input was (" << mat.m << "x" << mat.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  zgbmatrix mat_cp(mat);
  zgematrix mat_inv(mat.m,mat.n);
  mat_inv.identity();
  mat_cp.zgbsv(mat_inv);
  
  return _(mat_inv);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! return its conjugate matrix */
inline _zgbmatrix conj(const zgbmatrix& mat)
{VERBOSE_REPORT;
  zgbmatrix newmat(mat.m, mat.n, mat.kl, mat.ku);
  for(long i=0; i<mat.m; i++){
    for(long j=std::max(long(0),i-mat.kl); j<std::min(mat.n,i+mat.ku+1); j++){
      newmat(i,j) =std::conj(mat(i,j));
    }
  }
  return _(newmat);
}

//=============================================================================
/*! return its conjugate transposed zgbmatrix */
inline _zgbmatrix conjt(const zgbmatrix& mat)
{VERBOSE_REPORT;
  zgbmatrix newmat(mat.n, mat.m, mat.ku, mat.kl);
  for(long i=0; i<newmat.m; i++){
    for(long j=std::max(long(0),i-newmat.kl); j<std::min(newmat.n,i+newmat.ku+1); j++){
      newmat(i,j) =std::conj(mat(j,i));
    }
  }
  return _(newmat);
}
