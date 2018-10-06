//=============================================================================
/*! _zgematrix*double operator */
inline _zgematrix operator*(const _zgematrix& mat, const double& d)
{VERBOSE_REPORT;
  zdscal_(mat.m*mat.n, d, mat.array, 1);
  return mat;
}

//=============================================================================
/*! _zgematrix/double operator */
inline _zgematrix operator/(const _zgematrix& mat, const double& d)
{VERBOSE_REPORT;
  zdscal_(mat.m*mat.n, 1./d, mat.array, 1);
  return mat;
}
