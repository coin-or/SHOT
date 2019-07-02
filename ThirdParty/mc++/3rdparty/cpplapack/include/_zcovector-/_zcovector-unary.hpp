//=============================================================================
/*! +_zcovector operator */
inline const _zcovector& operator+(const _zcovector& vec)
{VERBOSE_REPORT;
  return vec;
}

//=============================================================================
/*! -_zcovector operator */
inline _zcovector operator-(const _zcovector& vec)
{VERBOSE_REPORT;
  for(long i=0; i<vec.l; i++){ vec.array[i]=-vec.array[i]; }
  return vec;
}
