//=============================================================================
/*! dgsmatrix*=double operator */
inline dgsmatrix& dgsmatrix::operator*=(const double& d)
{VERBOSE_REPORT;
  for(std::vector<dcomponent>::iterator it=data.begin(); it!=data.end(); it++){
    it->v *=d;
  }
  return *this;
}

//=============================================================================
/*! dgsmatrix/=double operator */
inline dgsmatrix& dgsmatrix::operator/=(const double& d)
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
/*! dgsmatrix*double operator */
inline _dgsmatrix operator*(const dgsmatrix& mat, const double& d)
{VERBOSE_REPORT;
  dgsmatrix newmat(mat);
  for(std::vector<dcomponent>::iterator it=newmat.data.begin(); it!=mat.data.end(); it++){
    it->v *=d;
  }
  return _(newmat);
}

//=============================================================================
/*! dgsmatrix/double operator */
inline _dgsmatrix operator/(const dgsmatrix& mat, const double& d)
{VERBOSE_REPORT;
  dgsmatrix newmat(mat);
  for(std::vector<dcomponent>::iterator it=newmat.data.begin(); it!=mat.data.end(); it++){
    it->v /=d;
  }
  return _(newmat);
}
