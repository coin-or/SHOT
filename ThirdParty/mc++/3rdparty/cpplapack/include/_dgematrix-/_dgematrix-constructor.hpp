//=============================================================================
/*! _dgematrix constructor without arguments */
inline _dgematrix::_dgematrix()
{VERBOSE_REPORT;
  m =0;
  n =0;
  array =NULL;
  darray =NULL;
}

//=============================================================================
/*! _dgematrix copy constructor */
inline _dgematrix::_dgematrix(const _dgematrix& mat)
{VERBOSE_REPORT;
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
/*! dgematrix destructor */
inline _dgematrix::~_dgematrix()
{VERBOSE_REPORT;
  delete [] darray;
  delete [] array;
}
