//=============================================================================
/*! comple*_zgbmatrix operator */
inline _zgbmatrix operator*(const comple& d, const _zgbmatrix& mat)
{VERBOSE_REPORT;
  zscal_((mat.kl+mat.ku+1)*mat.n, d, mat.array, 1);
  return mat;
}
