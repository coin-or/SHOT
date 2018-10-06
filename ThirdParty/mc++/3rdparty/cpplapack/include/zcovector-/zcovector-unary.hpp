//=============================================================================
/*! +zcovector operator */
inline const zcovector& operator+(const zcovector& vec)
{VERBOSE_REPORT;
  return vec;
}

//=============================================================================
/*! -zcovector operator */
inline _zcovector operator-(const zcovector& vec)
{VERBOSE_REPORT;
  zcovector newvec(vec.l);
  for(long i=0; i<newvec.l; i++){ newvec.array[i]=-vec.array[i]; }
  
  return _(newvec);
}
