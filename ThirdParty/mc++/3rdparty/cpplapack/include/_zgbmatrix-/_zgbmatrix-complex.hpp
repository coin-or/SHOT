//=============================================================================
/*! _zgbmatrix*comple operator */
inline _zgbmatrix operator*(const _zgbmatrix& mat, const comple& d)
{VERBOSE_REPORT;
  zscal_((mat.kl+mat.ku+1)*mat.n, d, mat.array, 1);
  return mat;
}

//=============================================================================
/*! _zgbmatrix/comple operator */
inline _zgbmatrix operator/(const _zgbmatrix& mat, const comple& d)
{VERBOSE_REPORT;
  zscal_((mat.kl+mat.ku+1)*mat.n, 1./d, mat.array, 1);
  return mat;
}
