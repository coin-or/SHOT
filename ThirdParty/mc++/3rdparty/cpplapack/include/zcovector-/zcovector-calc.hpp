//=============================================================================
/*! return a transposed row vector */
inline _zrovector t(const zcovector& covec)
{VERBOSE_REPORT;
  zrovector rovec(covec.l);
  zcopy_(covec.l, covec.array, 1, rovec.array, 1);
  
  return _(rovec);
}
//=============================================================================
/*! return its conjugated vector */
inline _zcovector conj(const zcovector& vec)
{VERBOSE_REPORT;
  zcovector newvec(vec.l);
  for(long i=0; i<vec.l; i++){ newvec(i) =std::conj(vec(i)); }
  
  return _(newvec);
}

//=============================================================================
/*! return a conjugate transposed row vector */
inline _zrovector conjt(const zcovector& covec)
{VERBOSE_REPORT;
  zrovector rovec(covec.l);
  for(long i=0; i<covec.l; i++){ rovec(i) =std::conj(covec(i)); }
  
  return _(rovec);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! return its Euclidean norm */
inline double nrm2(const zcovector& vec)
{VERBOSE_REPORT;
  return dznrm2_(vec.l, vec.array, 1);
}

//=============================================================================
/*! return the index of element having the largest absolute value
 in 0-based numbering system */
inline long idamax(const zcovector& vec)
{VERBOSE_REPORT;
  return izamax_(vec.l, vec.array, 1) -1;
}

//=============================================================================
/*! return its largest absolute value */
inline comple damax(const zcovector& vec)
{VERBOSE_REPORT;
  return vec.array[izamax_(vec.l, vec.array, 1) -1];
}
