//=============================================================================
/*! comple*_zcovector operator */
inline _zcovector operator*(const comple& d, const _zcovector& vec)
{VERBOSE_REPORT;
  zscal_(vec.l, d, vec.array, 1);  
  return vec;
}
