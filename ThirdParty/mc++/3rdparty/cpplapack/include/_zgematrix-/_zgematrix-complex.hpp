//=============================================================================
/*! _zgematrix*comple operator */
inline _zgematrix operator*(const _zgematrix& mat, const comple& d)
{VERBOSE_REPORT;
  zscal_(mat.m*mat.n, d, mat.array, 1);
  return mat;
}

//=============================================================================
/*! _zgematrix/comple operator */
inline _zgematrix operator/(const _zgematrix& mat, const comple& d)
{VERBOSE_REPORT;
  zscal_(mat.m*mat.n, 1./d, mat.array, 1);
  return mat;
}
