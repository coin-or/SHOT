//=============================================================================
/*! zgsmatrix*=comple operator */
inline zgsmatrix& zgsmatrix::operator*=(const comple& d)
{VERBOSE_REPORT;
  for(std::vector<zcomponent>::iterator it=data.begin(); it!=data.end(); it++){
    it->v *=d;
  }
  return *this;
}

//=============================================================================
/*! zgsmatrix/=comple operator */
inline zgsmatrix& zgsmatrix::operator/=(const comple& d)
{VERBOSE_REPORT;
  for(std::vector<zcomponent>::iterator it=data.begin(); it!=data.end(); it++){
    it->v /=d;
  }
  return *this;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! zgsmatrix*comple operator */
inline _zgsmatrix operator*(const zgsmatrix& mat, const comple& d)
{VERBOSE_REPORT;
  zgsmatrix newmat(mat);
  for(std::vector<zcomponent>::iterator it=newmat.data.begin(); it!=mat.data.end(); it++){
    it->v *=d;
  }
  return _(newmat);
}

//=============================================================================
/*! zgsmatrix/comple operator */
inline _zgsmatrix operator/(const zgsmatrix& mat, const comple& d)
{VERBOSE_REPORT;
  zgsmatrix newmat(mat);
  for(std::vector<zcomponent>::iterator it=newmat.data.begin(); it!=mat.data.end(); it++){
    it->v /=d;
  }
  return _(newmat);
}
