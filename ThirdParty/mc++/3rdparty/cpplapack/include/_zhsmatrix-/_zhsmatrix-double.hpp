//=============================================================================
/*! _zhsmatrix*double operator */
inline _zhsmatrix operator*(const _zhsmatrix& mat, const double& d)
{VERBOSE_REPORT;
  for(std::vector<zcomponent>::iterator it=mat.data.begin(); it!=mat.data.end(); it++){
    it->v *=d;
  }
  return mat;
}

//=============================================================================
/*! _zhsmatrix/double operator */
inline _zhsmatrix operator/(const _zhsmatrix& mat, const double& d)
{VERBOSE_REPORT;
  for(std::vector<zcomponent>::iterator it=mat.data.begin(); it!=mat.data.end(); it++){
    it->v /=d;
  }
  return mat;
}
