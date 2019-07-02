//=============================================================================
/*! +_zhsmatrix operator */
inline const _zhsmatrix& operator+(const _zhsmatrix& mat)
{VERBOSE_REPORT;
  return mat;
}

//=============================================================================
/*! -_zhsmatrix operator */
inline _zhsmatrix operator-(const _zhsmatrix& mat)
{VERBOSE_REPORT;
  for(std::vector<zcomponent>::iterator it=mat.data.begin(); it!=mat.data.end(); it++){
    it->v =-it->v;
  }
  
  return mat;
}
