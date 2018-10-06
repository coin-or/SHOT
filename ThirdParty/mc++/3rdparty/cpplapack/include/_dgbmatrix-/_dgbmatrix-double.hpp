//=============================================================================
/*! _dgbmatrix*double operator */
inline _dgbmatrix operator*(const _dgbmatrix& mat, const double& d)
{VERBOSE_REPORT;
  dscal_((mat.kl+mat.ku+1)*mat.n, d, mat.array, 1);
  return mat;
}

//=============================================================================
/*! _dgbmatrix/double operator */
inline _dgbmatrix operator/(const _dgbmatrix& mat, const double& d)
{VERBOSE_REPORT;
  dscal_((mat.kl+mat.ku+1)*mat.n, 1./d, mat.array, 1);
  return mat;
}
