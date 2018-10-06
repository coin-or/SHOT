//=============================================================================
/*! comple*_zrovector operator */
inline _zrovector operator*(const comple& d, const _zrovector& vec)
{VERBOSE_REPORT;
  zscal_(vec.l, d, vec.array, 1);  
  return vec;
}
