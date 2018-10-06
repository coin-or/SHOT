//=============================================================================
/*! nullify all the vector data */
inline void _drovector::nullify() const
{VERBOSE_REPORT;
  l=0;
  cap=0;
  array=NULL;
}

//=============================================================================
/*! destroy all the vector data */
inline void _drovector::destroy() const
{VERBOSE_REPORT;
  delete [] array;
  array=NULL;
}
