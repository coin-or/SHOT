//=============================================================================
/*! _zgematrix constructor without arguments */
inline _zgematrix::_zgematrix()
{VERBOSE_REPORT;
  //////// initialize ////////
  m =0;
  n =0;
  array =NULL;
  darray =NULL;
}

//=============================================================================
/*! _zgematrix copy constructor */
inline _zgematrix::_zgematrix(const _zgematrix& mat)
{VERBOSE_REPORT;
  //////// initialize ////////
  m =mat.m;
  n =mat.n;
  array =mat.array;
  darray =mat.darray;
  
  mat.nullify();
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! zgematrix destructor */
inline _zgematrix::~_zgematrix()
{VERBOSE_REPORT;
  delete [] array;
  delete [] darray;
}
