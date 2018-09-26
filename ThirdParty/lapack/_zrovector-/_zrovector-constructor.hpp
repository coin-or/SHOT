//=============================================================================
/*! zrovector constructor */
inline _zrovector::_zrovector()
{VERBOSE_REPORT;
  //////// initialize ////////
  l =0;
  array =NULL;
}

//=============================================================================
/*! _zrovector copy constructor */
inline _zrovector::_zrovector(const _zrovector& vec)
{VERBOSE_REPORT;
  //////// initialize ////////
  l =vec.l;
  array =vec.array;
  
  vec.nullify();
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! _zrovector destructor */
inline _zrovector::~_zrovector()
{VERBOSE_REPORT;
  delete [] array;
}
