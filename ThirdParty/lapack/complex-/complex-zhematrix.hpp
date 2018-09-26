//=============================================================================
/*! comple*zhematrix operator */
inline _zgematrix operator*(const comple& d, const zhematrix& mat)
{VERBOSE_REPORT;
  mat.complete();
  zgematrix newmat(mat.n, mat.n);
  for(long i=0; i<mat.n*mat.n; i++){ newmat.array[i] =d*mat.array[i]; }
  
  return _(newmat);
}
