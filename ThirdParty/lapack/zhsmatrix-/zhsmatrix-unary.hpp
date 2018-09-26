//=============================================================================
/*! +zhsmatrix operator */
inline const zhsmatrix& operator+(const zhsmatrix& mat)
{VERBOSE_REPORT;
  return mat;
}

//=============================================================================
/*! -zhsmatrix operator */
inline _zhsmatrix operator-(const zhsmatrix& mat)
{VERBOSE_REPORT;
  zhsmatrix newmat(mat);
  for(std::vector<zcomponent>::iterator it=newmat.data.begin(); it!=newmat.data.end(); it++){
    it->v =-it->v;
  }
  return _(newmat);
}
