//=============================================================================
/*! _dsymatrix*double operator */
inline _dsymatrix operator*(const _dsymatrix& mat, const double& d)
{VERBOSE_REPORT;
  for(long i=0; i<mat.n; i++){
    for(long j=0; j<=i; j++){
      mat.darray[j][i] *=d;
    }
  }
  return mat;
}

//=============================================================================
/*! dsymatrix/double operator */
inline _dsymatrix operator/(const _dsymatrix& mat, const double& d)
{VERBOSE_REPORT;
  for(long i=0; i<mat.n; i++){
    for(long j=0; j<=i; j++){
      mat.darray[j][i] /=d;
    }
  }
  return mat;
}
