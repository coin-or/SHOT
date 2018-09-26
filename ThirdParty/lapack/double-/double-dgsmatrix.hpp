//=============================================================================
/*! double*dgsmatrix operator */
inline _dgsmatrix operator*(const double& d, const dgsmatrix& mat)
{VERBOSE_REPORT;
  dgsmatrix newmat(mat);
  for(std::vector<dcomponent>::iterator it=newmat.data.begin(); it!=newmat.data.end(); it++){
    it->v *= d;
  }
  return _(newmat);
}
