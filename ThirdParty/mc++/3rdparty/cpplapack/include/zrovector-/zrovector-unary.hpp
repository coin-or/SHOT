//=============================================================================
/*! +zrovector operator */
inline const zrovector& operator+(const zrovector& vec)
{VERBOSE_REPORT;
  return vec;
}

//=============================================================================
/*! -zrovector operator */
inline _zrovector operator-(const zrovector& vec)
{VERBOSE_REPORT;
  zrovector newvec(vec.l);
  for(long i=0; i<newvec.l; i++){ newvec.array[i]=-vec.array[i]; }
  
  return _(newvec);
}
