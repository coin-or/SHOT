//=============================================================================
/*! nullify all the vector data */
inline void _zcovector::nullify() const
{VERBOSE_REPORT;
  l=0;
  array=NULL;
}

//=============================================================================
/*! destroy all the vector data */
inline void _zcovector::destroy() const
{VERBOSE_REPORT;
  delete [] array;
  array=NULL;
}
