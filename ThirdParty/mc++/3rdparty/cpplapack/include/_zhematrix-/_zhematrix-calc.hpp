//=============================================================================
/*! return transposed _zhematrix */
inline _zhematrix t(const _zhematrix& mat)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  WARNING_REPORT;
  std::cerr << "This function call has no effect since the matrix is symmetric." << std::endl;
#endif//CPPL_DEBUG
  
  return mat;
}

//=============================================================================
/*! return its inverse matrix */
inline _zgematrix i(const _zhematrix& mat)
{VERBOSE_REPORT;
  zhematrix mat_cp(mat);
  zgematrix mat_inv(mat_cp.n,mat_cp.n);
  mat_inv.identity();
  mat_cp.zhesv(mat_inv);
  
  return _(mat_inv);
}
