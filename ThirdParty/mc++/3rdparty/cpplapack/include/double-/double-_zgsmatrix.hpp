//=============================================================================
/*! double*_zgsmatrix operator */
inline _zgsmatrix operator*(const double& d, const _zgsmatrix& mat)
{VERBOSE_REPORT;
  for(std::vector<zcomponent>::iterator it=mat.data.begin(); it!=mat.data.end(); it++){
    it->v *= d;
  }
  
  return mat;
}
