//=============================================================================
/*! double*_zgbmatrix operator */
inline _zgbmatrix operator*(const double& d, const _zgbmatrix& mat)
{VERBOSE_REPORT;
  zdscal_((mat.kl+mat.ku+1)*mat.n, d, mat.array, 1);
  return mat;
}
