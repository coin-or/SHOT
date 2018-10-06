//=============================================================================
/*! +_zrovector operator */
inline const _zrovector& operator+(const _zrovector& vec)
{VERBOSE_REPORT;
  return vec;
}

//=============================================================================
/*! -_zrovector operator */
inline _zrovector operator-(const _zrovector& vec)
{VERBOSE_REPORT;
  for(long i=0; i<vec.l; i++){ vec.array[i]=-vec.array[i]; }
  return vec;
}
