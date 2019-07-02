//=============================================================================
/*! _dgsmatrix*double operator */
inline _dgsmatrix operator*(const _dgsmatrix& mat, const double& d)
{VERBOSE_REPORT;
  for(std::vector<dcomponent>::iterator it=mat.data.begin(); it!=mat.data.end(); it++){
    it->v *= d;
  }
  return mat;
}

//=============================================================================
/*! _dgsmatrix/double operator */
inline _dgsmatrix operator/(const _dgsmatrix& mat, const double& d)
{VERBOSE_REPORT;
  for(std::vector<dcomponent>::iterator it=mat.data.begin(); it!=mat.data.end(); it++){
    it->v /= d;
  }
  return mat;
}
