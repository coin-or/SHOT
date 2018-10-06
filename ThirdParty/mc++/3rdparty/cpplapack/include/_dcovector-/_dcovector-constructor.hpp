//=============================================================================
/*! _dcovector constructor */
inline _dcovector::_dcovector()
{VERBOSE_REPORT;VERBOSE_REPORT;
  l =0;
  cap =0;
  array =NULL;
}

//=============================================================================
/*! _dcovector copy constructor */
inline _dcovector::_dcovector(const _dcovector& vec)
{VERBOSE_REPORT;VERBOSE_REPORT;
  l =vec.l;
  cap =vec.cap;
  array =vec.array;
  
  vec.nullify();
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! _dcovector destructor */
inline _dcovector::~_dcovector()
{VERBOSE_REPORT;VERBOSE_REPORT;
  delete[] array;
}
