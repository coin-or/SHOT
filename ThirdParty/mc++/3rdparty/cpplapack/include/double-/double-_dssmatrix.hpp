//=============================================================================
/*! double*_dssmatrix operator */
inline _dssmatrix operator*(const double& d, const _dssmatrix& mat)
{VERBOSE_REPORT;
  for(std::vector<dcomponent>::iterator it=mat.data.begin(); it!=mat.data.end(); it++){
    it->v *=d;
  }
  return mat;
}
