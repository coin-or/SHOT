//=============================================================================
/*! dgematrix*=double operator */
inline dgematrix& dgematrix::operator*=(const double& d)
{VERBOSE_REPORT;
  dscal_(m*n, d, array, 1);
  return *this;
}

//=============================================================================
/*! dgematrix/=double operator */
inline dgematrix& dgematrix::operator/=(const double& d)
{VERBOSE_REPORT;
  dscal_(m*n, 1./d, array, 1);
  return *this;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! dgematrix*double operator */
inline _dgematrix operator*(const dgematrix& mat, const double& d)
{VERBOSE_REPORT;
  dgematrix newmat(mat.m, mat.n);
  for(long i=0; i<mat.m*mat.n; i++){ newmat.array[i] =mat.array[i]*d; }
  
  return _(newmat);
}

//=============================================================================
/*! dgematrix/double operator */
inline _dgematrix operator/(const dgematrix& mat, const double& d)
{VERBOSE_REPORT;
  dgematrix newmat(mat.m, mat.n);
  for(long i=0; i<mat.m*mat.n; i++){ newmat.array[i] =mat.array[i]/d; }
  
  return _(newmat);
}
