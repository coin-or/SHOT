//=============================================================================
/*! _dgematrix*double operator */
inline _dgematrix operator*(const _dgematrix& mat, const double& d)
{VERBOSE_REPORT;
  dscal_(mat.m*mat.n, d, mat.array, 1);
  return mat;
}

//=============================================================================
/*! _dgematrix/double operator */
inline _dgematrix operator/(const _dgematrix& mat, const double& d)
{VERBOSE_REPORT;
  dscal_(mat.m*mat.n, 1./d, mat.array, 1);
  return mat;
}
