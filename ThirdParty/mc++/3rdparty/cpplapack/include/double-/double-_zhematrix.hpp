//=============================================================================
/*! double*_zhematrix operator */
inline _zhematrix operator*(const double& d, const _zhematrix& mat)
{VERBOSE_REPORT;
  zdscal_(mat.n*mat.n, d, mat.array, 1);
  return mat;
}
