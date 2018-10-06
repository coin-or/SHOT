//=============================================================================
/*! cast to _zrovector */
inline _zrovector drovector::to_zrovector() const
{VERBOSE_REPORT;
  zrovector newvec(l);
  
  for(long i=0; i<l; i++){
    newvec.array[i] =comple(array[i], 0.);
  }
  
  return _(newvec);
}
