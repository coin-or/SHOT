//=============================================================================
/*! +_dsymatrix operator */
inline const _dsymatrix& operator+(const _dsymatrix& mat)
{VERBOSE_REPORT;
  return mat;
}

//=============================================================================
/*! -_dsymatrix operator */
inline _dsymatrix operator-(const _dsymatrix& mat)
{VERBOSE_REPORT;
  for(long i=0; i<mat.n*mat.n; i++){ mat.array[i]=-mat.array[i]; }
  return mat;
}
