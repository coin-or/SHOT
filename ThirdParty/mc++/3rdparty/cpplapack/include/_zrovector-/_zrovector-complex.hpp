//=============================================================================
/*! _zrovector*comple operator */
inline _zrovector operator*(const _zrovector& vec, const comple& d)
{VERBOSE_REPORT;
  zscal_(vec.l, d, vec.array, 1);  
  return vec;
}

//=============================================================================
/*! _zrovector/comple operator */
inline _zrovector operator/(const _zrovector& vec, const comple& d)
{VERBOSE_REPORT;
  zscal_(vec.l, 1./d, vec.array, 1);
  return vec;
}
