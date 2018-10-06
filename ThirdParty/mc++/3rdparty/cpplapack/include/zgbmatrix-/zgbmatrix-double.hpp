//=============================================================================
/*! zgbmatrix*=double operator */
inline zgbmatrix& zgbmatrix::operator*=(const double& d)
{VERBOSE_REPORT;
  zdscal_((kl+ku+1)*n, d, array, 1);
  return *this;
}

//=============================================================================
/*! zgbmatrix/=double operator */
inline zgbmatrix& zgbmatrix::operator/=(const double& d)
{VERBOSE_REPORT;
  zdscal_((kl+ku+1)*n, 1./d, array, 1);
  return *this;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! zgbmatrix*double operator */
inline _zgbmatrix operator*(const zgbmatrix& mat, const double& d)
{VERBOSE_REPORT;
  zgbmatrix newmat(mat.m, mat.n, mat.kl, mat.ku);
  for(long i=0; i<(newmat.kl+newmat.ku+1)*newmat.n; i++){
    newmat.array[i] =mat.array[i]*d;
  }
  
  return _(newmat);
}

//=============================================================================
/*! zgbmatrix/double operator */
inline _zgbmatrix operator/(const zgbmatrix& mat, const double& d)
{VERBOSE_REPORT;
  zgbmatrix newmat(mat.m, mat.n, mat.kl, mat.ku);
  for(long i=0; i<(newmat.kl+newmat.ku+1)*newmat.n; i++){
    newmat.array[i] =mat.array[i]/d;
  }
  
  return _(newmat);
}
