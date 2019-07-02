//=============================================================================
/*! cast to _zgematrix */
inline _zgematrix dgematrix::to_zgematrix() const
{VERBOSE_REPORT;
  zgematrix newmat(m,n);
  for(long i=0; i<m*n; i++){
    newmat.array[i] =comple(array[i],0.0);
  }
  
  return _(newmat);
}
