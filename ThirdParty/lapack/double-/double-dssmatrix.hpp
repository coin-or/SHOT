//=============================================================================
/*! double*dssmatrix operator */
inline _dssmatrix operator*(const double& d, const dssmatrix& mat)
{VERBOSE_REPORT;
  dssmatrix newmat(mat.n, mat.data.size());
  for(std::vector<dcomponent>::const_iterator it=mat.data.begin(); it!=mat.data.end(); it++){
    newmat.put(it->i, it->j, d*it->v);
  }
  return _(newmat);
}
