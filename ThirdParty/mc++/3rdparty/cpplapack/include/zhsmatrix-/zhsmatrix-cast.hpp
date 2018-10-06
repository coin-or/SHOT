//=============================================================================
/*! convert to _zgematrix */
inline _zgematrix zhsmatrix::to_zgematrix() const
{VERBOSE_REPORT;
  zgematrix newmat( zgematrix(m,n).zero() );
  
  for(std::vector<zcomponent>::const_iterator it=data.begin(); it!=data.end(); it++){
    newmat(it->i, it->j) =it->v;
    newmat(it->j, it->i) =std::conj(it->v);
  }
  
  return _(newmat);
}

//=============================================================================
/*! convert to _zhematrix */
inline _zhematrix zhsmatrix::to_zhematrix() const
{VERBOSE_REPORT;
  zhematrix newmat(n);
  newmat.zero();
  
  for(std::vector<zcomponent>::const_iterator it=data.begin(); it!=data.end(); it++){
    newmat(it->i, it->j) =it->v;
  }
  
  return _(newmat);
}

//=============================================================================
/*! convert to _zgsmatrix */
inline _zgsmatrix zhsmatrix::to_zgsmatrix() const
{VERBOSE_REPORT;
  zgsmatrix newmat(m,n);
  newmat.zero();
  
  for(std::vector<zcomponent>::const_iterator it=data.begin(); it!=data.end(); it++){
    newmat.put(it->i, it->j, it->v);
    if(it->i!=it->j){
      newmat.put(it->j, it->i, std::conj(it->v));
    }
  }
  
  return _(newmat);
}
