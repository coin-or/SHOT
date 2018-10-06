//=============================================================================
/*! double*zcovector operator */
inline _zcovector operator*(const double& d, const zcovector& vec)
{VERBOSE_REPORT;
  zcovector newvec(vec.l);
  for(long i=0; i<vec.l; i++){ newvec.array[i] =d*vec.array[i]; }
  return _(newvec);
}
