//=============================================================================
/*! +zgsmatrix operator */
inline const zgsmatrix& operator+(const zgsmatrix& mat)
{VERBOSE_REPORT;
  return mat;
}

//=============================================================================
/*! -zgsmatrix operator */
inline _zgsmatrix operator-(const zgsmatrix& mat)
{VERBOSE_REPORT;
  zgsmatrix newmat(mat);
  for(std::vector<zcomponent>::iterator it=newmat.data.begin(); it!=newmat.data.end(); it++){
    it->v =-it->v;
  }
  return _(newmat);
}
