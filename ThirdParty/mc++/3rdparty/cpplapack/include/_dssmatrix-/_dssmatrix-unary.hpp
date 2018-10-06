//=============================================================================
/*! +_dssmatrix operator */
inline const _dssmatrix& operator+(const _dssmatrix& mat)
{VERBOSE_REPORT;
  return mat;
}

//=============================================================================
/*! -_dssmatrix operator */
inline _dssmatrix operator-(const _dssmatrix& mat)
{VERBOSE_REPORT;
  for(std::vector<dcomponent>::iterator it=mat.data.begin(); it!=mat.data.end(); it++){
    it->v =-it->v;
  }
  
  return mat;
}
