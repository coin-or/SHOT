//=============================================================================
/*! convert to _zgematrix */
inline _zgematrix zgbmatrix::to_zgematrix() const
{VERBOSE_REPORT;
  zgematrix newmat( zgematrix(m,n).zero() );
  for(long i=0; i<m; i++){
    for(long j=std::max(long(0),i-kl); j<std::min(n,i+ku+1); j++){
      newmat(i,j) =(*this)(i,j);
    }
  }
  return _(newmat);
}
