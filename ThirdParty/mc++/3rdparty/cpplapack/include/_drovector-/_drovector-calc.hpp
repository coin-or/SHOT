//=============================================================================
/*! return a transposed column vector */
inline _dcovector t(const _drovector& rovec)
{VERBOSE_REPORT;
  _dcovector covec;
  covec.l =rovec.l;
  covec.cap =rovec.cap;
  delete [] covec.array;
  covec.array =rovec.array;
  
  rovec.nullify();
  return covec;
}

//=============================================================================
/*! return its Euclidean norm */
inline double nrm2(const _drovector& vec)
{VERBOSE_REPORT;
  double val( dnrm2_(vec.l, vec.array, 1) );
  vec.destroy();
  return val;
}

//=============================================================================
/*! return the index of element having the largest absolute value
  in 0-based numbering system */
inline long idamax(const _drovector& vec)
{VERBOSE_REPORT;
  long i( idamax_(vec.l, vec.array, 1) -1 );
  vec.destroy();
  return i;
}

//=============================================================================
/*! return its largest absolute value */
inline double damax(const _drovector& vec)
{VERBOSE_REPORT;
  double val( vec.array[idamax_(vec.l, vec.array, 1)-1] );
  vec.destroy();
  return val;
}
