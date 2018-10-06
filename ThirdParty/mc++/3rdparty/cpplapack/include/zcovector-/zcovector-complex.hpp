//=============================================================================
/*! zcovector*=comple operator */
inline zcovector& zcovector::operator*=(const comple& d)
{VERBOSE_REPORT;
  zscal_(l, d, array, 1);
  return *this;
}

//=============================================================================
/*! zcovector/=comple operator */
inline zcovector& zcovector::operator/=(const comple& d)
{VERBOSE_REPORT;
  zscal_(l, 1./d, array, 1);
  return *this;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! zcovector*comple operator */
inline _zcovector operator*(const zcovector& vec, const comple& d)
{VERBOSE_REPORT;
  zcovector newvec(vec.l);
  for(long i=0; i<vec.l; i++){ newvec.array[i] =vec.array[i]*d; }
  
  return _(newvec);
}

//=============================================================================
/*! zcovector/comple operator */
inline _zcovector operator/(const zcovector& vec, const comple& d)
{VERBOSE_REPORT;
  zcovector newvec(vec.l);
  for(long i=0; i<vec.l; i++){ newvec.array[i] =vec.array[i]/d; }
  
  return _(newvec);
}
