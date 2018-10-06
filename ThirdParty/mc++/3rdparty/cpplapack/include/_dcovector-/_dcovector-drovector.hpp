//=============================================================================
/*! _dcovector*drovector operator */
inline _dgematrix operator*(const _dcovector& covec, const drovector& rovec)
{VERBOSE_REPORT;
  dgematrix newmat(covec.l, rovec.l);
  
  for(long i=0; i<newmat.m; i++){
    for(long j=0; j<newmat.n; j++){
      newmat(i,j) =covec(i)*rovec(j);
    }
  }
  
  covec.destroy();
  return _(newmat);
}
