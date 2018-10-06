//=============================================================================
/*! cast to _zhematrix */
inline _zhematrix _dsymatrix::to_zhematrix() const
{VERBOSE_REPORT;
  zhematrix newmat(n);
  for(long i=0; i<n; i++){
    for(long j=0; j<=i; j++){
      newmat(i,j) =comple((*this)(i,j),0.0);
    }
  }
  
  destroy();
  return _(newmat);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! convert to _dgematrix */
inline _dgematrix _dsymatrix::to_dgematrix() const
{VERBOSE_REPORT;
  dgematrix newmat(n,n);
  for(long i=0; i<n; i++){
    for(long j=0; j<n; j++){
      newmat(i,j) =(*this)(i,j);
    }
  }
  
  destroy();
  return _(newmat);
}
