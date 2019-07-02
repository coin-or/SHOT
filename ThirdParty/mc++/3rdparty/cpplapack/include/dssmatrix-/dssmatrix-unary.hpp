//=============================================================================
/*! +dssmatrix operator */
inline const dssmatrix& operator+(const dssmatrix& mat)
{VERBOSE_REPORT;
  return mat;
}

//=============================================================================
/*! -dssmatrix operator */
inline _dssmatrix operator-(const dssmatrix& mat)
{VERBOSE_REPORT;
  dssmatrix newmat(mat);
  for(std::vector<dcomponent>::iterator it=newmat.data.begin(); it!=newmat.data.end(); it++){
    it->v =-it->v;
  }
  
  return _(newmat);
}
