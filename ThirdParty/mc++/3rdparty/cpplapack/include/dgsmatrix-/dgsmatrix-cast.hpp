//=============================================================================
/*! cast to _zgsmatrix */
inline _zgsmatrix dgsmatrix::to_zgsmatrix() const
{VERBOSE_REPORT;
  zgsmatrix newmat(m,n,data.size());
  for(std::vector<dcomponent>::const_iterator it=data.begin(); it!=data.end(); it++){
    newmat.put(it->i, it->j, comple(it->v,0.));
  }
  return _(newmat);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! convert to _dgematrix */
inline _dgematrix dgsmatrix::to_dgematrix() const
{VERBOSE_REPORT;
  dgematrix newmat(m,n);
  newmat.zero();
  for(std::vector<dcomponent>::const_iterator it=data.begin(); it!=data.end(); it++){
    newmat(it->i,it->j) = it->v;
  }
  return _(newmat);
}
