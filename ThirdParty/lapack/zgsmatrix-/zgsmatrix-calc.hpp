//=============================================================================
/*! return transposed zgsmatrix */
inline _zgsmatrix t(const zgsmatrix& mat)
{VERBOSE_REPORT;
  zgsmatrix newmat(mat);
  std::swap(newmat.rows,newmat.cols);
  
  for(std::vector<zcomponent>::iterator it=newmat.data.begin(); it!=newmat.data.end(); it++){
    std::swap(it->i,it->j);
  }
  
  return _(newmat);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! return its conjugate matrix */
inline _zgsmatrix conj(const zgsmatrix& mat)
{VERBOSE_REPORT;
  zgsmatrix newmat(mat);
  
  for(std::vector<zcomponent>::iterator it=newmat.data.begin(); it!=newmat.data.end(); it++){
    it->v =std::conj(it->v);
  }
  
  return _(newmat);
}

//=============================================================================
/*! return its conjugate transposed matrix */
inline _zgsmatrix conjt(const zgsmatrix& mat)
{VERBOSE_REPORT;
  zgsmatrix newmat(mat);
  std::swap(newmat.rows,newmat.cols);
  
  for(std::vector<zcomponent>::iterator it=newmat.data.begin(); it!=newmat.data.end(); it++){
    std::swap(it->i,it->j);
    it->v =std::conj(it->v);
  }
  
  return _(newmat);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! search the index of element having the largest absolute value in 0-based numbering system */
inline void idamax(long& i, long& j, const zgsmatrix& mat)
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
}

//=============================================================================
/*! return its largest absolute value */
inline comple damax(const zgsmatrix& mat)
{VERBOSE_REPORT;
  std::vector<zcomponent>::const_iterator itx(mat.data.begin());
  double vmax =0.;
  for(std::vector<zcomponent>::const_iterator it=mat.data.begin(); it!=mat.data.end(); it++){
    if( vmax < norm(it->v) ){
      vmax =norm(it->v);
      itx =it;
    }
  }
  return itx->v;
}
