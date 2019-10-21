//=============================================================================
/*! return its transposed matrix */
inline _zhsmatrix t(const _zhsmatrix& mat)
{VERBOSE_REPORT;
  for(std::vector<zcomponent>::iterator it=mat.data.begin(); it!=mat.data.end(); it++){
    it->v =std::conj(it->v);
  }
  return mat;
}

//=============================================================================
/*! return its conjugate matrix */
inline _zhsmatrix conj(const _zhsmatrix& mat)
{VERBOSE_REPORT;
  for(std::vector<zcomponent>::iterator it=mat.data.begin(); it!=mat.data.end(); it++){
    it->v =std::conj(it->v);
  }
  return mat;
}

//=============================================================================
/*! return its conjugate transposed matrix */
inline _zhsmatrix conjt(const _zhsmatrix& mat)
{VERBOSE_REPORT;
  return mat;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! search the index of element having the largest absolute value
  in 0-based numbering system */
inline void idamax(long& i, long& j, const _zhsmatrix& mat)
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
inline comple damax(const _zhsmatrix& mat)
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
