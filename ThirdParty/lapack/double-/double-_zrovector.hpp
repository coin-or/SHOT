//=============================================================================
/*! double*_zrovector operator */
inline _zrovector operator*(const double& d, const _zrovector& vec)
{VERBOSE_REPORT;
  zdscal_(vec.l, d, vec.array, 1);
  return vec;
}
