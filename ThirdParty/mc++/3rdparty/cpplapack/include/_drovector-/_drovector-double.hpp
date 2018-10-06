//=============================================================================
/*! _drovector*double operator */
inline _drovector operator*(const _drovector& vec, const double& d)
{VERBOSE_REPORT;
  dscal_(vec.l, d, vec.array, 1);
  return vec;
}

//=============================================================================
/*! _drovector/double operator */
inline _drovector operator/(const _drovector& vec, const double& d)
{VERBOSE_REPORT;
  dscal_(vec.l, 1./d, vec.array, 1);
  return vec;
}
