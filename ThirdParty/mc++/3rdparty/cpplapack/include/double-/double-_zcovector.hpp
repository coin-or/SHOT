//=============================================================================
/*! double*_zcovector operator */
inline _zcovector operator*(const double& d, const _zcovector& vec)
{VERBOSE_REPORT;
  zdscal_(vec.l, d, vec.array, 1);
  return vec;
}
