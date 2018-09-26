//=============================================================================
/*! drovector constructor */
inline _drovector::_drovector()
{VERBOSE_REPORT;
  l =0;
  cap =0;
  array =NULL;
}

//=============================================================================
/*! _drovector copy constructor */
inline _drovector::_drovector(const _drovector& vec)
{VERBOSE_REPORT;
  l =vec.l;
  cap =vec.cap;
  array =vec.array;
  
  vec.nullify();
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! _drovector destructor */
inline _drovector::~_drovector()
{VERBOSE_REPORT;
  delete[] array; 
}
