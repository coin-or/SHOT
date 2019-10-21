//=============================================================================
/*! _dcovector*double operator */
inline _dcovector operator*(const _dcovector& vec, const double& d)
{VERBOSE_REPORT;
  dscal_(vec.l, d, vec.array, 1);
  return vec;
}

//=============================================================================
/*! _dcovector/double operator */
inline _dcovector operator/(const _dcovector& vec, const double& d)
{VERBOSE_REPORT;
  dscal_(vec.l, 1./d, vec.array, 1);
  return vec;
}
