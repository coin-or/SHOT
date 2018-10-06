//=============================================================================
/*! double*zgsmatrix operator */
inline _zgsmatrix operator*(const double& d, const zgsmatrix& mat)
{VERBOSE_REPORT;
  zgsmatrix newmat(mat);
  
  for(std::vector<zcomponent>::iterator it=newmat.data.begin(); it!=newmat.data.end(); it++){
    it->v *= d;
  }
  
  return _(newmat);
}
