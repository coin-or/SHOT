//=============================================================================
/*! _zhematrix*double operator */
inline _zhematrix operator*(const _zhematrix& mat, const double& d)
{VERBOSE_REPORT;
  zdscal_(mat.n*mat.n, d, mat.array, 1);
  return mat;
}

//=============================================================================
/*! _zhematrix/double operator */
inline _zhematrix operator/(const _zhematrix& mat, const double& d)
{VERBOSE_REPORT;
  zdscal_(mat.n*mat.n, 1./d, mat.array, 1);
  return mat;
}
