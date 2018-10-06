//=============================================================================
/*! double*zrovector operator */
inline _zrovector operator*(const double& d, const zrovector& vec)
{VERBOSE_REPORT;
  zrovector newvec(vec.l);
  for(long i=0; i<vec.l; i++){ newvec.array[i] =d*vec.array[i]; }  
  return _(newvec);
}
