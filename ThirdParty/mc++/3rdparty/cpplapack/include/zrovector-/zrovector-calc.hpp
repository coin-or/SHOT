//=============================================================================
/*! return a transposed column vector */
inline _zcovector t(const zrovector& rovec)
{VERBOSE_REPORT;
  zcovector covec(rovec.l);
  zcopy_(rovec.l, rovec.array, 1, covec.array, 1);
  
  return _(covec);
}

//=============================================================================
/*! return its conjugated vector */
inline _zrovector conj(const zrovector& vec)
{VERBOSE_REPORT;
  zrovector newvec(vec.l);
  
  for(long i=0; i<vec.l; i++){
    newvec(i) =std::conj(vec(i));
  }
  
  return _(newvec);
}

//=============================================================================
/*! return a conjugate transposed column vector */
inline _zcovector conjt(const zrovector& rovec)
{VERBOSE_REPORT;
  zcovector covec(rovec.l);
  
  for(long i=0; i<rovec.l; i++){
    covec(i) =std::conj(rovec(i));
  }
  
  return _(covec);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! return its Euclidean norm */
inline double nrm2(const zrovector& vec)
{VERBOSE_REPORT;
  return dznrm2_(vec.l, vec.array, 1);
}

//=============================================================================
/*! return the index of element having the largest absolute value
  in 0-based numbering system */
inline long idamax(const zrovector& vec)
{VERBOSE_REPORT;
  return izamax_(vec.l, vec.array, 1) -1;
}

//=============================================================================
/*! return its largest absolute value */
inline comple damax(const zrovector& vec)
{VERBOSE_REPORT;
  return vec.array[izamax_(vec.l, vec.array, 1) -1];
}
