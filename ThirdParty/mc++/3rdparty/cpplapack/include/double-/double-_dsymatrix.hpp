//=============================================================================
/*! double*_dsymatrix operator */
inline _dsymatrix operator*(const double& d, const _dsymatrix& mat)
{VERBOSE_REPORT;
  for(long i=0; i<mat.n; i++){
    for(long j=0; j<=i; j++){
      mat.darray[j][i] *=d;
    }
  }
  
  return mat;
}
