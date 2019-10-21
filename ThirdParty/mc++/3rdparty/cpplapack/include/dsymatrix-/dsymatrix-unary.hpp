//=============================================================================
/*! +dsymatrix operator */
inline const dsymatrix& operator+(const dsymatrix& mat)
{VERBOSE_REPORT;
  return mat;
}

//=============================================================================
/*! -dsymatrix operator */
inline _dsymatrix operator-(const dsymatrix& mat)
{VERBOSE_REPORT;
  dsymatrix newmat(mat.n);
  for(long i=0; i<newmat.n*newmat.n; i++){ newmat.array[i]=-mat.array[i]; }
  
  return _(newmat);
}
