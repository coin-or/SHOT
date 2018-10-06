//=============================================================================
/*! dcovector*=double operator */
inline dcovector& dcovector::operator*=(const double& d)
{VERBOSE_REPORT;
  dscal_(l, d, array, 1);
  return *this;
}

//=============================================================================
/*! dcovector/=double operator */
inline dcovector& dcovector::operator/=(const double& d)
{VERBOSE_REPORT;
  dscal_(l, 1./d, array, 1);
  return *this;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! dcovector*double operator */
inline _dcovector operator*(const dcovector& vec, const double& d)
{VERBOSE_REPORT;
  dcovector newvec(vec.l);
  for(long i=0; i<vec.l; i++){ newvec.array[i] =vec.array[i]*d; }
  
  return _(newvec);
}

//=============================================================================
/*! dcovector/double operator */
inline _dcovector operator/(const dcovector& vec, const double& d)
{VERBOSE_REPORT;
  dcovector newvec(vec.l);
  for(long i=0; i<vec.l; i++){ newvec.array[i] =vec.array[i]/d; }
  
  return _(newvec);
}
