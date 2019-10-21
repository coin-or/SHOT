//=============================================================================
/*! double*_dcovector operator */
inline _dcovector operator*(const double& d, const _dcovector& vec)
{VERBOSE_REPORT;
  dscal_(vec.l, d, vec.array, 1);
  return vec;
}
