//=============================================================================
/*! _zcovector*double operator */
inline _zcovector operator*(const _zcovector& vec, const double& d)
{VERBOSE_REPORT;
  zdscal_(vec.l, d, vec.array, 1);
  return vec;
}

//=============================================================================
/*! _zcovector/double operator */
inline _zcovector operator/(const _zcovector& vec, const double& d)
{VERBOSE_REPORT;
  zdscal_(vec.l, 1./d, vec.array, 1);
  return vec;
}
