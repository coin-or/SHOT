//=============================================================================
/*! double*_zhsmatrix operator */
inline _zhsmatrix operator*(const double& d, const _zhsmatrix& mat)
{VERBOSE_REPORT;
  for(std::vector<zcomponent>::iterator it=mat.data.begin(); it!=mat.data.end(); it++){
    it->v *=d;
  }
  return mat;
}
