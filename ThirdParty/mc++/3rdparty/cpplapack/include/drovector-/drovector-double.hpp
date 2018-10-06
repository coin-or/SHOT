//=============================================================================
/*! drovector*=double operator */
inline drovector& drovector::operator*=(const double& d)
{VERBOSE_REPORT;
  dscal_(l, d, array, 1);
  return *this;
}

//=============================================================================
/*! drovector/=double operator */
inline drovector& drovector::operator/=(const double& d)
{VERBOSE_REPORT;
  dscal_(l, 1./d, array, 1);
  return *this;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! drovector*double operator */
inline _drovector operator*(const drovector& vec, const double& d)
{VERBOSE_REPORT;
  drovector newvec(vec.l);
  for(long i=0; i<vec.l; i++){ newvec.array[i] =vec.array[i]*d; }
  
  return _(newvec);
}

//=============================================================================
/*! drovector/double operator */
inline _drovector operator/(const drovector& vec, const double& d)
{VERBOSE_REPORT;
  drovector newvec(vec.l);
  for(long i=0; i<vec.l; i++){ newvec.array[i] =vec.array[i]/d; }
  
  return _(newvec);
}
