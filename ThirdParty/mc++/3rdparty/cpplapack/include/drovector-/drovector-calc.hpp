//=============================================================================
/*! return a transposed column vector */
inline _dcovector t(const drovector& rovec)
{VERBOSE_REPORT;
  dcovector covec(rovec.l);
  dcopy_(rovec.l, rovec.array, 1, covec.array, 1);
  
  return _(covec);
}

//=============================================================================
/*! return its Euclidean norm */
inline double nrm2(const drovector& vec)
{VERBOSE_REPORT;
  return dnrm2_(vec.l, vec.array, 1);
}

//=============================================================================
/*! return the index of element having the largest absolute value
  in 0-based numbering system */
inline long idamax(const drovector& vec)
{VERBOSE_REPORT;
  return idamax_(vec.l, vec.array, 1) -1;
}

//=============================================================================
/*! return its largest absolute value */
inline double damax(const drovector& vec)
{VERBOSE_REPORT;
  return vec.array[idamax_(vec.l, vec.array, 1) -1];
}
