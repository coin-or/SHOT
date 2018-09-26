//=============================================================================
/*! return transposed zgematrix */
inline _zhematrix t(const zhematrix& mat)
{VERBOSE_REPORT;
  zhematrix newmat(mat.n);
  for(long i=0; i<newmat.n; i++){ for(long j=0; j<=i; j++){
    newmat(i,j) =mat(j,i);
  }}
  
  return _(newmat);
}

//=============================================================================
/*! return its inverse matrix */
inline _zgematrix i(const zhematrix& mat)
{VERBOSE_REPORT;
  zhematrix mat_cp(mat);
  zgematrix mat_inv(mat.n,mat.n);
  mat_inv.identity();
  mat_cp.zhesv(mat_inv);
  
  return _(mat_inv);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! return its conjugate matrix */
inline _zhematrix conj(const zhematrix& mat)
{VERBOSE_REPORT;
  zhematrix newmat(mat.n);
  
  for(long i=0; i<mat.n; i++){
    for(long j=0; j<=i; j++){
      newmat(i,j) =std::conj(mat(i,j));
    }
  }
  
  return _(newmat);
}

//=============================================================================
/*! return its conjugate transposed matrix */
inline _zhematrix conjt(const zhematrix& mat)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  WARNING_REPORT;
  std::cerr << "This function call has no effect since the matrix is Hermitian." << std::endl;
#endif//CPPL_DEBUG
  
  zhematrix newmat(mat);
  return _(newmat);
}
