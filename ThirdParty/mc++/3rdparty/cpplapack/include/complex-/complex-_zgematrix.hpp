//=============================================================================
/*! comple*_zgematrix operator */
inline _zgematrix operator*(const comple& d, const _zgematrix& mat)
{VERBOSE_REPORT;
  zscal_(mat.m*mat.n, d, mat.array, 1);  
  return mat;
}
