//=============================================================================
/*! double*zhsmatrix operator */
inline _zhsmatrix operator*(const double& d, const zhsmatrix& mat)
{VERBOSE_REPORT;
  zhsmatrix newmat(mat);
  
  for(std::vector<zcomponent>::iterator it=newmat.data.begin(); it!=newmat.data.end(); it++){
    it->v *=d;
  }
  
  return _(newmat);
}
