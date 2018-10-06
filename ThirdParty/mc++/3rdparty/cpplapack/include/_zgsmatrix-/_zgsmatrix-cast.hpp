//=============================================================================
/*! convert to _zgematrix */
inline _zgematrix _zgsmatrix::to_zgematrix() const
{VERBOSE_REPORT;
  zgematrix newmat(m,n);
  newmat.zero();
  
  for(std::vector<zcomponent>::const_iterator it=data.begin(); it!=data.end(); it++){
    newmat(it->i,it->j) = it->v;
  }
  
  destroy();
  return _(newmat);
}
