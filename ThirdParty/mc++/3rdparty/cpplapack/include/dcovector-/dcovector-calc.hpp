//=============================================================================
/*! return a transposed row vector */
inline _drovector t(const dcovector& covec)
{VERBOSE_REPORT;
  drovector rovec(covec.l);
  dcopy_(covec.l, covec.array, 1, rovec.array, 1);
  
  return _(rovec);
}

//=============================================================================
/*! return its Euclidean norm */
inline double nrm2(const dcovector& vec)
{VERBOSE_REPORT;
  return dnrm2_(vec.l, vec.array, 1);
}

//=============================================================================
/*! return the index of element having the largest absolute value
 in 0-based numbering system */
inline long idamax(const dcovector& vec)
{VERBOSE_REPORT;
  return idamax_(vec.l, vec.array, 1) -1;
}

//=============================================================================
/*! return its largest absolute value */
inline double damax(const dcovector& vec)
{VERBOSE_REPORT;
  return vec.array[idamax_(vec.l, vec.array, 1) -1];
}
