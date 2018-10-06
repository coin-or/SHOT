//=============================================================================
/*! double*zgematrix operator */
inline _zgematrix operator*(const double& d, const zgematrix& mat)
{VERBOSE_REPORT;
  zgematrix newmat(mat.m, mat.n);
  for(long i=0; i<mat.m*mat.n; i++){ newmat.array[i] =d*mat.array[i]; }  
  return _(newmat);
}
