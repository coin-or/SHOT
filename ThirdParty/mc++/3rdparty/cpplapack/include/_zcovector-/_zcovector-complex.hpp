//=============================================================================
/*! _zcovector*comple operator */
inline _zcovector operator*(const _zcovector& vec, const comple& d)
{VERBOSE_REPORT;
  zscal_(vec.l, d, vec.array, 1);
  return vec;
}

//=============================================================================
/*! _zcovector/comple operator */
inline _zcovector operator/(const _zcovector& vec, const comple& d)
{VERBOSE_REPORT;
  zscal_(vec.l, 1./d, vec.array, 1);
  return vec;
}
