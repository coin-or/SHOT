//=============================================================================
/*! double*_dgsmatrix operator */
inline _dgsmatrix operator*(const double& d, const _dgsmatrix& mat)
{VERBOSE_REPORT;
  for(std::vector<dcomponent>::iterator it=mat.data.begin(); it!=mat.data.end(); it++){
    it->v *= d;
  }
  return mat;
}
