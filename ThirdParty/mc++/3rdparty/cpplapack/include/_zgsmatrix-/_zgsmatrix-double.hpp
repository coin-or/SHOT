//=============================================================================
/*! _zgsmatrix*double operator */
inline _zgsmatrix operator*(const _zgsmatrix& mat, const double& d)
{VERBOSE_REPORT;
  for(std::vector<zcomponent>::iterator it=mat.data.begin(); it!=mat.data.end(); it++){
    it->v *=d;
  }
  return mat;
}

//=============================================================================
/*! _zgsmatrix/double operator */
inline _zgsmatrix operator/(const _zgsmatrix& mat, const double& d)
{VERBOSE_REPORT;
  for(std::vector<zcomponent>::iterator it=mat.data.begin(); it!=mat.data.end(); it++){
    it->v /=d;
  }
  return mat;
}
