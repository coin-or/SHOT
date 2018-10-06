//=============================================================================
/*! +_dgsmatrix operator */
inline const _dgsmatrix& operator+(const _dgsmatrix& mat)
{VERBOSE_REPORT;
  return mat;
}

//=============================================================================
/*! -_dgsmatrix operator */
inline _dgsmatrix operator-(const _dgsmatrix& mat)
{VERBOSE_REPORT;
  for(std::vector<dcomponent>::iterator it=mat.data.begin(); it!=mat.data.end(); it++){
    it->v = -it->v;
  }
  return mat;
}
