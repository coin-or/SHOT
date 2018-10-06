//=============================================================================
/*! double*drovector operator */
inline _drovector operator*(const double& d, const drovector& vec)
{VERBOSE_REPORT;
  drovector newvec(vec.l);
  for(long i=0; i<vec.l; i++){ newvec.array[i] =d*vec.array[i]; }  
  return _(newvec);
}
