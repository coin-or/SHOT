//=============================================================================
/*! return transposed dgsmatrix */
inline _dgsmatrix t(const dgsmatrix& mat)
{VERBOSE_REPORT;
  dgsmatrix newmat(mat);
  for(std::vector<dcomponent>::iterator it=newmat.data.begin(); it!=newmat.data.end(); it++){
    std::swap(it->i,it->j);
  }
  std::swap(newmat.rows,newmat.cols);
  
  return _(newmat);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! search the index of element having the largest absolute value
  in 0-based numbering system */
inline void idamax(long& i, long& j, const dgsmatrix& mat)
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
inline double damax(const dgsmatrix& mat)
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
