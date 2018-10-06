//=============================================================================
/*! cast to _zhematrix */
inline _zhematrix dsymatrix::to_zhematrix() const
{VERBOSE_REPORT;
  zhematrix newmat(n);
  for(long i=0; i<n; i++){
    for(long j=0; j<=i; j++){
      newmat(i,j) =comple((*this)(i,j),0.0);
    }
  }
  
  return _(newmat);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! convert to _dgematrix */
inline _dgematrix dsymatrix::to_dgematrix() const
{VERBOSE_REPORT;
  dgematrix newmat(n,n);
  for(long i=0; i<n; i++){
    for(long j=0; j<n; j++){
      newmat(i,j) =(*this)(i,j);
    }
  }
  
  return _(newmat);
}

//=============================================================================
/*! convert to _dssmatrix */
inline _dssmatrix dsymatrix::to_dssmatrix(const double eps) const
{VERBOSE_REPORT;
  dssmatrix newmat(n);
  for(long i=0; i<n; i++){
    for(long j=0; j<=i; j++){
      if( fabs((*this)(i,j))>eps ){ newmat(i,j) =(*this)(i,j); }
    }
  }
  
  return _(newmat);
}
