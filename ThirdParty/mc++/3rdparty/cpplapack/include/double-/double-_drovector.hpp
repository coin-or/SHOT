//=============================================================================
/*! double*_drovector operator */
inline _drovector operator*(const double& d, const _drovector& vec)
{VERBOSE_REPORT;
  dscal_(vec.l, d, vec.array, 1);  
  return vec;
}
