//=============================================================================
/*! return a transposed row vector */
inline drovector t(const _dcovector& covec)
{VERBOSE_REPORT;
  _drovector rovec;
  rovec.l =covec.l;
  rovec.cap =covec.cap;
  delete [] rovec.array;
  rovec.array =covec.array;

  covec.nullify();
  return rovec;
}

//=============================================================================
/*! return its Euclidean norm */
inline double nrm2(const _dcovector& vec)
{VERBOSE_REPORT;
  double val( dnrm2_(vec.l, vec.array, 1) );
  vec.destroy();
  return val;
}

//=============================================================================
/*! return the index of element having the largest absolute value
 in 0-based numbering system */
inline long idamax(const _dcovector& vec)
{VERBOSE_REPORT;
  long i( idamax_(vec.l, vec.array, 1) -1 );
  vec.destroy();
  return i;
}

//=============================================================================
/*! return its largest absolute value */
inline double damax(const _dcovector& vec)
{VERBOSE_REPORT;
  double val( vec.array[idamax_(vec.l, vec.array, 1)-1] );
  vec.destroy();
  return val;
}
