//=============================================================================
/*! double*dsymatrix operator */
inline _dsymatrix operator*(const double& d, const dsymatrix& mat)
{VERBOSE_REPORT;
  dsymatrix newmat(mat.n);
  for(long i=0; i<mat.n; i++){
    for(long j=0; j<=i; j++){
      newmat.darray[j][i] =d*mat.darray[j][i];
    }
  }
  return _(newmat);
}
