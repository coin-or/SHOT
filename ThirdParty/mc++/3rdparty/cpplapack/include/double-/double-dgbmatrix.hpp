//=============================================================================
/*! double*dgbmatrix operator */
inline _dgbmatrix operator*(const double& d, const dgbmatrix& mat)
{VERBOSE_REPORT;
  dgbmatrix newmat(mat.m, mat.n, mat.kl, mat.ku);
  for(long i=0; i<(newmat.kl+newmat.ku+1)*newmat.n; i++){
    newmat.array[i] =d*mat.array[i];
  }
  return _(newmat);
}
