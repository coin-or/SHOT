//=============================================================================
/*! _dssmatrix*double operator */
inline _dssmatrix operator*(const _dssmatrix& mat, const double& d)
{VERBOSE_REPORT;
  for(std::vector<dcomponent>::iterator it=mat.data.begin(); it!=mat.data.end(); it++){
    it->v *=d;
  }
  
  return mat;
}

//=============================================================================
/*! _dssmatrix/double operator */
inline _dssmatrix operator/(const _dssmatrix& mat, const double& d)
{VERBOSE_REPORT;
  for(std::vector<dcomponent>::iterator it=mat.data.begin(); it!=mat.data.end(); it++){
    it->v /=d;
  }
  
  return mat;
}
