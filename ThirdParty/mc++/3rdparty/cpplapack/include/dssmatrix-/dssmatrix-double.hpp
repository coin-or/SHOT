//=============================================================================
/*! dssmatrix*=double operator */
inline dssmatrix& dssmatrix::operator*=(const double& d)
{VERBOSE_REPORT;
  for(std::vector<dcomponent>::iterator it=data.begin(); it!=data.end(); it++){
    it->v *=d;
  }
  return *this;
}

//=============================================================================
/*! dssmatrix/=double operator */
inline dssmatrix& dssmatrix::operator/=(const double& d)
{VERBOSE_REPORT;
  for(std::vector<dcomponent>::iterator it=data.begin(); it!=data.end(); it++){
    it->v /=d;
  }
  return *this;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! dssmatrix*double operator */
inline _dssmatrix operator*(const dssmatrix& mat, const double& d)
{VERBOSE_REPORT;
  dssmatrix newmat(mat);
  for(std::vector<dcomponent>::iterator it=newmat.data.begin(); it!=mat.data.end(); it++){
    it->v *=d;
  }
  return _(newmat);
}

//=============================================================================
/*! dssmatrix/double operator */
inline _dssmatrix operator/(const dssmatrix& mat, const double& d)
{VERBOSE_REPORT;
  dssmatrix newmat(mat);
  for(std::vector<dcomponent>::iterator it=newmat.data.begin(); it!=mat.data.end(); it++){
    it->v /=d;
  }
  return _(newmat);
}
