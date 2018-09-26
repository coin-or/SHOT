//=============================================================================
/*! nullify all the vector data */
inline void _dcovector::nullify() const
{VERBOSE_REPORT;
  l=0;
  cap=0;
  array=NULL;
}

//=============================================================================
/*!  destroy all the vector data */
inline void _dcovector::destroy() const
{VERBOSE_REPORT;
  delete [] array;
  array=NULL;
}
