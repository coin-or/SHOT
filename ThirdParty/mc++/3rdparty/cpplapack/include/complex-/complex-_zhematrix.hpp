//=============================================================================
/*! comple*_zhematrix operator */
inline _zgematrix operator*(const comple& d, const _zhematrix& mat)
{VERBOSE_REPORT;
  zgematrix newmat( mat.to_zgematrix() );
  zscal_(mat.n*mat.n, d, newmat.array, 1);
  
  mat.destroy();
  return _(newmat);
}
