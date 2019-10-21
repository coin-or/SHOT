//=============================================================================
/*! return transposed zgematrix */
inline _zgematrix t(const _zgematrix& mat)
{VERBOSE_REPORT;
  zgematrix newmat(mat.n,mat.m);
  
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
inline _zgematrix i(const _zgematrix& mat)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(mat.m!=mat.n){
    ERROR_REPORT;
    std::cerr << "This matrix is not square and has no inverse matrix." << std::endl
              << "Your input was (" << mat.m << "x" << mat.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  zgematrix mat_cp(mat);
  zgematrix mat_inv(mat_cp.m,mat_cp.n);
  mat_inv.identity();
  mat_cp.zgesv(mat_inv);
  
  return _(mat_inv);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! return its conjugate matrix */
inline _zgematrix conj(const _zgematrix& mat)
{VERBOSE_REPORT;
  for(long i=0; i<mat.m; i++){ for(long j=0; j<mat.n; j++){
    mat(i,j) =std::conj(mat(i,j));
  }}
  
  return mat;
}

//=============================================================================
/*! return its conjugate transposed matrix */
inline _zgematrix conjt(const _zgematrix& mat)
{VERBOSE_REPORT;
  zgematrix newmat(mat.n,mat.m);
  for(long i=0; i<newmat.m; i++){
    for(long j=0; j<newmat.n; j++){
      newmat(i,j) =std::conj(mat(j,i));
    }
  }
  
  mat.destroy();
  return _(newmat);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! search the index of element having the largest absolute value
  in 0-based numbering system */
inline void idamax(long& i, long& j, const _zgematrix& mat)
{VERBOSE_REPORT;
  long index( izamax_(mat.m*mat.n, mat.array, 1) -1 );
  i =index%mat.m;
  j =index/mat.m;
  
  mat.destroy();
}

//=============================================================================
/*! return its largest absolute value */
inline comple damax(const _zgematrix& mat)
{VERBOSE_REPORT;
  comple val( mat.array[izamax_(mat.m*mat.n, mat.array, 1) -1] );
  
  mat.destroy();
  return val;
}
