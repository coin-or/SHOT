//=============================================================================
/*! _zrovector*double operator */
inline _zrovector operator*(const _zrovector& vec, const double& d)
{VERBOSE_REPORT;
  zdscal_(vec.l, d, vec.array, 1);
  return vec;
}

//=============================================================================
/*! _zrovector/double operator */
inline _zrovector operator/(const _zrovector& vec, const double& d)
{VERBOSE_REPORT;
  zdscal_(vec.l, 1./d, vec.array, 1);
  return vec;
}
