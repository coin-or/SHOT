//=============================================================================
/*! zcovector*=double operator */
inline zcovector& zcovector::operator*=(const double& d)
{VERBOSE_REPORT;
  zdscal_(l, d, array, 1);
  return *this;
}

//=============================================================================
/*! zcovector/=double operator */
inline zcovector& zcovector::operator/=(const double& d)
{VERBOSE_REPORT;
  zdscal_(l, 1./d, array, 1);
  return *this;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! zcovector*double operator */
inline _zcovector operator*(const zcovector& vec, const double& d)
{VERBOSE_REPORT;
  zcovector newvec(vec.l);
  for(long i=0; i<vec.l; i++){ newvec.array[i] =vec.array[i]*d; }
  
  return _(newvec);
}

//=============================================================================
/*! zcovector/double operator */
inline _zcovector operator/(const zcovector& vec, const double& d)
{VERBOSE_REPORT;
  zcovector newvec(vec.l);
  for(long i=0; i<vec.l; i++){ newvec.array[i] =vec.array[i]/d; }
  
  return _(newvec);
}
