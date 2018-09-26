//=============================================================================
/*! +dgematrix operator */
inline const dgematrix& operator+(const dgematrix& mat)
{VERBOSE_REPORT;
  return mat;
}

//=============================================================================
/*! -dgematrix operator */
inline _dgematrix operator-(const dgematrix& mat)
{VERBOSE_REPORT;
  dgematrix newmat(mat.m,mat.n);
  for(long i=0; i<newmat.m*newmat.n; i++){ newmat.array[i]=-mat.array[i]; }
  return _(newmat);
}
