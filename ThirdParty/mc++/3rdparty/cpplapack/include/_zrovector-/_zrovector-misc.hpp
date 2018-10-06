//=============================================================================
/*! nullify all the vector data */
inline void _zrovector::nullify() const
{VERBOSE_REPORT;
  l=0;
  array=NULL;
}

//=============================================================================
/*! destroy all the vector data */
inline void _zrovector::destroy() const
{VERBOSE_REPORT;
  delete [] array;
  array=NULL;
}
