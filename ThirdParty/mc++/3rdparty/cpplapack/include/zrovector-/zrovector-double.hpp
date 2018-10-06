//=============================================================================
/*! zrovector*=double operator */
inline zrovector& zrovector::operator*=(const double& d)
{VERBOSE_REPORT;
  zdscal_(l, d, array, 1);
  return *this;
}

//=============================================================================
/*! zrovector/=double operator */
inline zrovector& zrovector::operator/=(const double& d)
{VERBOSE_REPORT;
  zdscal_(l, 1./d, array, 1);
  return *this;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! zrovector*double operator */
inline _zrovector operator*(const zrovector& vec, const double& d)
{VERBOSE_REPORT;
  zrovector newvec(vec.l);
  for(long i=0; i<vec.l; i++){ newvec.array[i] =vec.array[i]*d; }
  
  return _(newvec);
}

//=============================================================================
/*! zrovector/double operator */
inline _zrovector operator/(const zrovector& vec, const double& d)
{VERBOSE_REPORT;
  zrovector newvec(vec.l);
  for(long i=0; i<vec.l; i++){ newvec.array[i] =vec.array[i]/d; }
  
  return _(newvec);
}
