//=============================================================================
/*! return transposed dssmatrix */
inline _dssmatrix t(const dssmatrix& mat)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  WARNING_REPORT;
  std::cerr << "This function call has no effect since the matrix is symmetric." << std::endl;
#endif//CPPL_DEBUG

  dssmatrix newmat(mat);
  return _(newmat);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! search the index of element having the largest absolute value
  in 0-based numbering system */
inline void idamax(long& i, long& j, const dssmatrix& mat)
{VERBOSE_REPORT;
  std::vector<dcomponent>::const_iterator itx(mat.data.begin());
  double vmax =0.;
  for(std::vector<dcomponent>::const_iterator it=mat.data.begin(); it!=mat.data.end(); it++){
    if( vmax < fabs(it->v) ){
      vmax =fabs(it->v);
      itx =it;
    }
  }
  i =itx->i;
  j =itx->j;
}

//=============================================================================
/*! return its largest absolute value */
inline double damax(const dssmatrix& mat)
{VERBOSE_REPORT;
  std::vector<dcomponent>::const_iterator itx(mat.data.begin());
  double vmax =0.;
  for(std::vector<dcomponent>::const_iterator it=mat.data.begin(); it!=mat.data.end(); it++){
    if( vmax < fabs(it->v) ){
      vmax =fabs(it->v);
      itx =it;
    }
  }
  return itx->v;
}
