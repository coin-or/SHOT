//=============================================================================
/*! double*_dgematrix operator */
inline _dgematrix operator*(const double& d, const _dgematrix& mat)
{VERBOSE_REPORT;
  dscal_(mat.m*mat.n, d, mat.array, 1);
  return mat;
}
