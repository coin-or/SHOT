//=============================================================================
/*! convert to _zgematrix */
inline _zgematrix _zhematrix::to_zgematrix() const
{VERBOSE_REPORT;
  zgematrix newmat(n,n);
  for(long i=0; i<n; i++){
    for(long j=0; j<n; j++){
      newmat(i,j) =(*this)(i,j);
    }
  }
  
  destroy();
  return _(newmat);
}
