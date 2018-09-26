//=============================================================================
/*! +_dgematrix operator */
inline const _dgematrix& operator+(const _dgematrix& mat)
{VERBOSE_REPORT;
  return mat;
}

//=============================================================================
/*! -_dgematrix operator */
inline _dgematrix operator-(const _dgematrix& mat)
{VERBOSE_REPORT;
  for(long i=0; i<mat.m*mat.n; i++){ mat.array[i]=-mat.array[i]; }
  
  return mat;
}
