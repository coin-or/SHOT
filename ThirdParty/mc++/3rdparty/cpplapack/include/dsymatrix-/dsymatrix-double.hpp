//=============================================================================
/*! dsymatrix*=double operator */
inline dsymatrix& dsymatrix::operator*=(const double& d)
{VERBOSE_REPORT;
  for(long i=0; i<n; i++){
    for(long j=0; j<=i; j++){
      darray[j][i] *=d;
    }
  }
  
  return *this;
}

//=============================================================================
/*! dsymatrix/=double operator */
inline dsymatrix& dsymatrix::operator/=(const double& d)
{VERBOSE_REPORT;
  for(long i=0; i<n; i++){
    for(long j=0; j<=i; j++){
      darray[j][i] /=d;
    }
  }
  return *this;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! dsymatrix*double operator */
inline _dsymatrix operator*(const dsymatrix& mat, const double& d)
{VERBOSE_REPORT;
  dsymatrix newmat(mat.n);
  for(long i=0; i<mat.n; i++){
    for(long j=0; j<=i; j++){
      newmat.darray[j][i] =mat.darray[j][i]*d;
    }
  }
  
  return _(newmat);
}

//=============================================================================
/*! dsymatrix/double operator */
inline _dsymatrix operator/(const dsymatrix& mat, const double& d)
{VERBOSE_REPORT;
  dsymatrix newmat(mat.n);
  for(long i=0; i<mat.n; i++){
    for(long j=0; j<=i; j++){
      newmat.darray[j][i] =mat.darray[j][i]/d;
    }
  }
  
  return _(newmat);
}
