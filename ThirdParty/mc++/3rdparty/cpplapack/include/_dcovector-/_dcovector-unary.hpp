//=============================================================================
/*! +_dcovector operator */
inline const _dcovector& operator+(const _dcovector& vec)
{VERBOSE_REPORT;
  return vec;
}

//=============================================================================
/*! -_dcovector operator */
inline _dcovector operator-(const _dcovector& vec)
{VERBOSE_REPORT;
  for(long i=0; i<vec.l; i++){ vec.array[i]=-vec.array[i]; }
  
  return vec;
}
