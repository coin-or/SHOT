//=============================================================================
/*! zhematrix*comple operator */
inline _zgematrix operator*(const zhematrix& mat, const comple& d)
{VERBOSE_REPORT;
  mat.complete();
  zgematrix newmat(mat.n, mat.n);
  for(long i=0; i<mat.n*mat.n; i++){ newmat.array[i] =mat.array[i]*d; }
  
  return _(newmat);
}

//=============================================================================
/*! zhematrix/comple operator */
inline _zgematrix operator/(const zhematrix& mat, const comple& d)
{VERBOSE_REPORT;
  mat.complete();
  zgematrix newmat(mat.n, mat.n);
  for(long i=0; i<mat.n*mat.n; i++){ newmat.array[i] =mat.array[i]/d; }
  
  return _(newmat);
}
