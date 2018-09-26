//=============================================================================
/*! +_zgsmatrix operator */
inline const _zgsmatrix& operator+(const _zgsmatrix& mat)
{VERBOSE_REPORT;
  return mat;
}

//=============================================================================
/*! -_zgsmatrix operator */
inline _zgsmatrix operator-(const _zgsmatrix& mat)
{VERBOSE_REPORT;
  for(std::vector<zcomponent>::iterator it=mat.data.begin(); it!=mat.data.end(); it++){
    it->v = -it->v;
  }
  
  return mat;
}
