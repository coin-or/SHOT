//=============================================================================
/*! _zgbmatrix*double operator */
inline _zgbmatrix operator*(const _zgbmatrix& mat, const double& d)
{VERBOSE_REPORT;
  zdscal_((mat.kl+mat.ku+1)*mat.n, d, mat.array, 1);
  return mat;
}

//=============================================================================
/*! _zgbmatrix/double operator */
inline _zgbmatrix operator/(const _zgbmatrix& mat, const double& d)
{VERBOSE_REPORT;
  zdscal_((mat.kl+mat.ku+1)*mat.n, 1./d, mat.array, 1);
  return mat;
}
