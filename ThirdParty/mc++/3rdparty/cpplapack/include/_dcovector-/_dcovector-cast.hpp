//=============================================================================
/*! cast to _zcovector */
inline _zcovector _dcovector::to_zcovector() const
{VERBOSE_REPORT;
  zcovector newvec(l);
  
  for(long i=0; i<l; i++){
    newvec.array[i] =comple(array[i], 0.);
  }
  
  destroy();
  return _(newvec);
}
