//=============================================================================
/*! zgsmatrix*=double operator */
inline zgsmatrix& zgsmatrix::operator*=(const double& d)
{VERBOSE_REPORT;
  for(std::vector<zcomponent>::iterator it=data.begin(); it!=data.end(); it++){
    it->v *=d;
  }
  return *this;
}

//=============================================================================
/*! zgsmatrix/=double operator */
inline zgsmatrix& zgsmatrix::operator/=(const double& d)
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
/*! zgsmatrix*double operator */
inline _zgsmatrix operator*(const zgsmatrix& mat, const double& d)
{VERBOSE_REPORT;
  zgsmatrix newmat(mat);
  for(std::vector<zcomponent>::iterator it=newmat.data.begin(); it!=mat.data.end(); it++){
    it->v *=d;
  }
  return _(newmat);
}

//=============================================================================
/*! zgsmatrix/double operator */
inline _zgsmatrix operator/(const zgsmatrix& mat, const double& d)
{VERBOSE_REPORT;
  zgsmatrix newmat(mat);
  for(std::vector<zcomponent>::iterator it=newmat.data.begin(); it!=mat.data.end(); it++){
    it->v /=d;
  }
  return _(newmat);
}
