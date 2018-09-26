//=============================================================================
/*! return transposed dgematrix */
inline _dgematrix t(const _dgematrix& mat)
{VERBOSE_REPORT;
  dgematrix newmat(mat.n,mat.m);
  
  for(long i=0; i<newmat.m; i++){
    for(long j=0; j<newmat.n; j++){
      newmat(i,j) =mat(j,i);
    }
  }
  
  mat.destroy();
  return _(newmat);
}

//=============================================================================
/*! return its inverse matrix */
inline _dgematrix i(const _dgematrix& mat)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(mat.m!=mat.n){
    ERROR_REPORT;
    std::cerr << "This matrix is not square and has no inverse matrix." << std::endl
              << "Your input was (" << mat.m << "x" << mat.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  dgematrix mat_cp(mat);
  dgematrix mat_inv(mat_cp.m,mat_cp.n);
  mat_inv.identity();
  mat_cp.dgesv(mat_inv);
  
  return _(mat_inv);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! search the index of element having the largest absolute value
  in 0-based numbering system */
inline void idamax(long& i, long& j, const _dgematrix& mat)
{VERBOSE_REPORT;
  long index( idamax_(mat.m*mat.n, mat.array, 1) -1 );
  i =index%mat.m;
  j =index/mat.m;
  
  mat.destroy();
}

//=============================================================================
/*! return its largest absolute value */
inline double damax(const _dgematrix& mat)
{VERBOSE_REPORT;
  double val( mat.array[idamax_(mat.m*mat.n, mat.array, 1) -1] );
  
  mat.destroy();
  return val;
}
