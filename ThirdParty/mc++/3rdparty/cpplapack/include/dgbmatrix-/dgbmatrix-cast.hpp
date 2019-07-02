//=============================================================================
/*! cast to _zgbmatrix */
inline _zgbmatrix dgbmatrix::to_zgbmatrix() const
{VERBOSE_REPORT;
  zgbmatrix newmat(m,n,kl,ku);
  for(long i=0; i<(kl+ku+1)*n; i++){
    newmat.array[i] =comple(array[i],0.0);
  }
  
  return _(newmat);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! convert to _dgematrix */
inline _dgematrix dgbmatrix::to_dgematrix() const
{VERBOSE_REPORT;
  dgematrix newmat( dgematrix(m,n).zero() );
  for(long i=0; i<m; i++){
    for(long j=std::max(long(0),i-kl); j<std::min(n,i+ku+1); j++){
      newmat(i,j) =(*this)(i,j);
    }
  }
  
  return _(newmat);
}
