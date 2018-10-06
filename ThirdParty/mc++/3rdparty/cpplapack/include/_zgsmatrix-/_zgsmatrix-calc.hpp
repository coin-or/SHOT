//=============================================================================
/*! return transposed _zgsmatrix */
inline _zgsmatrix t(const _zgsmatrix& mat)
{VERBOSE_REPORT;
  for(std::vector<zcomponent>::iterator it=mat.data.begin(); it!=mat.data.end(); it++){
    std::swap(it->i,it->j);
  }
  std::swap(mat.rows,mat.cols);
  
  return mat;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! search the index of element having the largest absolute value in 0-based numbering system */
inline void idamax(long& i, long& j, const _zgsmatrix& mat)
{VERBOSE_REPORT;
  std::vector<zcomponent>::const_iterator itx(mat.data.begin());
  double vmax =0.;
  for(std::vector<zcomponent>::const_iterator it=mat.data.begin(); it!=mat.data.end(); it++){
    if( vmax < norm(it->v) ){
      vmax =norm(it->v);
      itx =it;
    }
  }
  i=itx->i;
  j=itx->j;
  
  mat.destroy();
}

//=============================================================================
/*! return its largest absolute value */
inline comple damax(const _zgsmatrix& mat)
{VERBOSE_REPORT;
  std::vector<zcomponent>::const_iterator itx(mat.data.begin());
  double vmax =0.;
  for(std::vector<zcomponent>::const_iterator it=mat.data.begin(); it!=mat.data.end(); it++){
    if( vmax < norm(it->v) ){
      vmax =norm(it->v);
      itx =it;
    }
  }
  
  mat.destroy();
  return itx->v;
}
