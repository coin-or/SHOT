//=============================================================================
/*! +dcovector operator */
inline const dcovector& operator+(const dcovector& vec)
{VERBOSE_REPORT;
  return vec;
}

//=============================================================================
/*! -dcovector operator */
inline _dcovector operator-(const dcovector& vec)
{VERBOSE_REPORT;
  dcovector newvec(vec.l);
  for(long i=0; i<newvec.l; i++){ newvec.array[i]=-vec.array[i]; }
  
  return _(newvec);
}
