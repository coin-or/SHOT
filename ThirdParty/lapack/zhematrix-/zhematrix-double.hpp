//=============================================================================
/*! zhematrix*=double operator */
inline zhematrix& zhematrix::operator*=(const double& d)
{VERBOSE_REPORT;
  zdscal_(n*n, d, array, 1);
  return *this;
}

//=============================================================================
/*! zhematrix/=double operator */
inline zhematrix& zhematrix::operator/=(const double& d)
{VERBOSE_REPORT;
  zdscal_(n*n, 1./d, array, 1);
  return *this;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! zhematrix*double operator */
inline _zhematrix operator*(const zhematrix& mat, const double& d)
{VERBOSE_REPORT;
  zhematrix newmat(mat.n);
  for(long i=0; i<mat.n*mat.n; i++){ newmat.array[i] =mat.array[i]*d; }
  
  return _(newmat);
}

//=============================================================================
/*! zhematrix/double operator */
inline _zhematrix operator/(const zhematrix& mat, const double& d)
{VERBOSE_REPORT;
  zhematrix newmat(mat.n);
  for(long i=0; i<mat.n*mat.n; i++){ newmat.array[i] =mat.array[i]/d; }
  
  return _(newmat);
}
