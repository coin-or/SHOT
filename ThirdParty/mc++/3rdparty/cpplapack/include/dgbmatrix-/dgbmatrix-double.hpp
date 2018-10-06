//=============================================================================
/*! dgbmatrix*=double operator */
inline dgbmatrix& dgbmatrix::operator*=(const double& d)
{VERBOSE_REPORT;
  dscal_((kl+ku+1)*n, d, array, 1);
  return *this;
}

//=============================================================================
/*! dgbmatrix/=double operator */
inline dgbmatrix& dgbmatrix::operator/=(const double& d)
{VERBOSE_REPORT;
  dscal_((kl+ku+1)*n, 1./d, array, 1);
  return *this;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! dgbmatrix*double operator */
inline _dgbmatrix operator*(const dgbmatrix& mat, const double& d)
{VERBOSE_REPORT;
  dgbmatrix newmat(mat.m, mat.n, mat.kl, mat.ku);
  for(long i=0; i<(newmat.kl+newmat.ku+1)*newmat.n; i++){
    newmat.array[i] =mat.array[i]*d;
  }
  
  return _(newmat);
}

//=============================================================================
/*! dgbmatrix/double operator */
inline _dgbmatrix operator/(const dgbmatrix& mat, const double& d)
{VERBOSE_REPORT;
  dgbmatrix newmat(mat.m, mat.n, mat.kl, mat.ku);
  for(long i=0; i<(newmat.kl+newmat.ku+1)*newmat.n; i++){
    newmat.array[i] =mat.array[i]/d;
  }
  
  return _(newmat);
}
