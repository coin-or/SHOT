//=============================================================================
/*! double*_zgematrix operator */
inline _zgematrix operator*(const double& d, const _zgematrix& mat)
{VERBOSE_REPORT;
  zdscal_(mat.m*mat.n, d, mat.array, 1);
  return mat;
}
