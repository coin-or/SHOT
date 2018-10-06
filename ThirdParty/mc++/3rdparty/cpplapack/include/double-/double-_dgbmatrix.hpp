//=============================================================================
/*! double*_dgbmatrix operator */
inline _dgbmatrix operator*(const double& d, const _dgbmatrix& mat)
{VERBOSE_REPORT;
  dscal_((mat.kl+mat.ku+1)*mat.n, d, mat.array, 1);
  return mat;
}
