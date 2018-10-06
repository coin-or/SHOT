//=============================================================================
/*! zrovector*=comple operator */
inline zrovector& zrovector::operator*=(const comple& d)
{VERBOSE_REPORT;
  zscal_(l, d, array, 1);
  return *this;
}

//=============================================================================
/*! zrovector/=comple operator */
inline zrovector& zrovector::operator/=(const comple& d)
{VERBOSE_REPORT;
  zscal_(l, 1./d, array, 1);
  return *this;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! zrovector*comple operator */
inline _zrovector operator*(const zrovector& vec, const comple& d)
{VERBOSE_REPORT;
  zrovector newvec(vec.l);
  for(long i=0; i<vec.l; i++){ newvec.array[i] =vec.array[i]*d; }
  
  return _(newvec);
}

//=============================================================================
/*! zrovector/comple operator */
inline _zrovector operator/(const zrovector& vec, const comple& d)
{VERBOSE_REPORT;
  zrovector newvec(vec.l);
  for(long i=0; i<vec.l; i++){ newvec.array[i] =vec.array[i]/d; }
  
  return _(newvec);
}
