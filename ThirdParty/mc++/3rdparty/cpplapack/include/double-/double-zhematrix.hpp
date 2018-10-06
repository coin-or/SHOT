//=============================================================================
/*! double*zhematrix operator */
inline _zhematrix operator*(const double& d, const zhematrix& mat)
{VERBOSE_REPORT;
  zhematrix newmat(mat.n);
  for(long i=0; i<mat.n*mat.n; i++){ newmat.array[i] =d*mat.array[i]; }  
  return _(newmat);
}
