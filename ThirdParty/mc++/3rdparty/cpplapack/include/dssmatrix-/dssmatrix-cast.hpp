//=============================================================================
/*! cast to _zhsmatrix */
inline _zhsmatrix dssmatrix::to_zhsmatrix() const
{VERBOSE_REPORT;
  zhsmatrix newmat(n,data.size());
  for(std::vector<dcomponent>::const_iterator it=data.begin(); it!=data.end(); it++){
    newmat.put(it->i, it->j, comple(it->v,0.0));
  }
  
  return _(newmat);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! convert to _dgematrix */
inline _dgematrix dssmatrix::to_dgematrix() const
{VERBOSE_REPORT;
  dgematrix newmat(m,n);
  newmat.zero();
  for(std::vector<dcomponent>::const_iterator it=data.begin(); it!=data.end(); it++){
    newmat(it->i, it->j) =it->v;
    newmat(it->j, it->i) =it->v;
  }
  
  return _(newmat);
}

//=============================================================================
/*! convert to _dsymatrix */
inline _dsymatrix dssmatrix::to_dsymatrix() const
{VERBOSE_REPORT;
  dsymatrix newmat(n);
  newmat.zero();
  for(std::vector<dcomponent>::const_iterator it=data.begin(); it!=data.end(); it++){
    newmat(it->i, it->j) =it->v;
  }
  
  return _(newmat);
}

//=============================================================================
/*! convert to _dgsmatrix */
inline _dgsmatrix dssmatrix::to_dgsmatrix() const
{VERBOSE_REPORT;
  dgsmatrix newmat( dgsmatrix(m,n,data.size()*2) );
  newmat.zero();
  for(std::vector<dcomponent>::const_iterator it=data.begin(); it!=data.end(); it++){
    newmat.put(it->i, it->j, it->v);
    if(it->i!=it->j){
      newmat.put(it->j, it->i, it->v);
    }
  }
  
  return _(newmat);
}
