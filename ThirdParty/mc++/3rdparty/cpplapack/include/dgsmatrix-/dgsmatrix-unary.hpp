//=============================================================================
/*! +dgsmatrix operator */
inline const dgsmatrix& operator+(const dgsmatrix& mat)
{VERBOSE_REPORT;
  return mat;
}

//=============================================================================
/*! -dgsmatrix operator */
inline _dgsmatrix operator-(const dgsmatrix& mat)
{VERBOSE_REPORT;
  dgsmatrix newmat(mat);
  for(std::vector<dcomponent>::iterator it=newmat.data.begin(); it!=newmat.data.end(); it++){
    it->v =-it->v;
  }
  return _(newmat);
}
