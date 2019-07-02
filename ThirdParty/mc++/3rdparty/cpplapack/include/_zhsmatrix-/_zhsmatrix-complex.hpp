//=============================================================================
/*! _zhsmatrix*comple operator */
inline _zgsmatrix operator*(const _zhsmatrix& mat, const comple& d)
{VERBOSE_REPORT;
  zgsmatrix newmat( mat.to_zgsmatrix() );
  return newmat*d;
}

//=============================================================================
/*! _zhsmatrix/comple operator */
inline _zgsmatrix operator/(const _zhsmatrix& mat, const comple& d)
{VERBOSE_REPORT;
  zgsmatrix newmat( mat.to_zgsmatrix() );
  return newmat/d;
}
