//=============================================================================
/*! zgematrix*=comple operator */
inline zgematrix& zgematrix::operator*=(const comple& d)
{VERBOSE_REPORT;
  zscal_(m*n, d, array, 1);
  return *this;
}

//=============================================================================
/*! zgematrix/=comple operator */
inline zgematrix& zgematrix::operator/=(const comple& d)
{VERBOSE_REPORT;
  zscal_(m*n, 1./d, array, 1);
  return *this;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! zgematrix*comple operator */
inline _zgematrix operator*(const zgematrix& mat, const comple& d)
{VERBOSE_REPORT;
  zgematrix newmat(mat.m, mat.n);
  for(long i=0; i<mat.m*mat.n; i++){ newmat.array[i] =mat.array[i]*d; }
  
  return _(newmat);
}

//=============================================================================
/*! zgematrix/comple operator */
inline _zgematrix operator/(const zgematrix& mat, const comple& d)
{VERBOSE_REPORT;
  zgematrix newmat(mat.m, mat.n);
  for(long i=0; i<mat.m*mat.n; i++){ newmat.array[i] =mat.array[i]/d; }
  
  return _(newmat);
}
