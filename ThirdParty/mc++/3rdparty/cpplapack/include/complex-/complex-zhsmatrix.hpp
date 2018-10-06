//=============================================================================
/*! complex*zhsmatrix operator */
inline _zgsmatrix operator*(const comple& d, const zhsmatrix& mat)
{VERBOSE_REPORT;
  zgsmatrix newmat( mat.to_zgsmatrix() );
  return d*newmat;
}
