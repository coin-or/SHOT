//=============================================================================
/*! _zhematrix*comple operator */
inline _zgematrix operator*(const _zhematrix& mat, const comple& d)
{VERBOSE_REPORT;
  zgematrix newmat( mat.to_zgematrix() );
  zscal_(mat.n*mat.n, d, newmat.array, 1);
  
  return _(newmat);
}

//=============================================================================
/*! zhematrix/comple operator */
inline _zgematrix operator/(const _zhematrix& mat, const comple& d)
{VERBOSE_REPORT;
  zgematrix newmat( mat.to_zgematrix() );
  zscal_(mat.n*mat.n, 1./d, newmat.array, 1);
  
  return _(newmat);
}
