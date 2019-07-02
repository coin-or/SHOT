//=============================================================================
/*! zhsmatrix*=double operator */
inline zhsmatrix& zhsmatrix::operator*=(const double& d)
{VERBOSE_REPORT;
  for(std::vector<zcomponent>::iterator it=data.begin(); it!=data.end(); it++){
    it->v *=d;
  }
  return *this;
}

//=============================================================================
/*! zhsmatrix/=double operator */
inline zhsmatrix& zhsmatrix::operator/=(const double& d)
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
/*! zhsmatrix*double operator */
inline _zhsmatrix operator*(const zhsmatrix& mat, const double& d)
{VERBOSE_REPORT;
  zhsmatrix newmat(mat);
  
  for(std::vector<zcomponent>::iterator it=newmat.data.begin(); it!=newmat.data.end(); it++){
    it->v *=d;
  }
  
  return _(newmat);
}

//=============================================================================
/*! zhsmatrix/double operator */
inline _zhsmatrix operator/(const zhsmatrix& mat, const double& d)
{VERBOSE_REPORT;
  zhsmatrix newmat(mat);
  
  for(std::vector<zcomponent>::iterator it=newmat.data.begin(); it!=newmat.data.end(); it++){
    it->v /=d;
  }
  
  return _(newmat);
}
