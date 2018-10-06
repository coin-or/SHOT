//=============================================================================
/*! +drovector operator */
inline const drovector& operator+(const drovector& vec)
{VERBOSE_REPORT;
  return vec;
}

//=============================================================================
/*! -drovector operator */
inline _drovector operator-(const drovector& vec)
{VERBOSE_REPORT;
  drovector newvec(vec.l);
  for(long i=0; i<newvec.l; i++){ newvec.array[i]=-vec.array[i]; }
  
  return _(newvec);
}
