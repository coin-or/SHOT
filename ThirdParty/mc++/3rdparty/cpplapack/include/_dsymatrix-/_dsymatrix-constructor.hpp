//=============================================================================
/*! _dsymatrix constructor without arguments */
inline _dsymatrix::_dsymatrix()
  :m(n)
{VERBOSE_REPORT;
  n =0;
  array =NULL;
  darray =NULL;
}

//=============================================================================
/*! _dsymatrix copy constructor */
inline _dsymatrix::_dsymatrix(const _dsymatrix& mat)
  :m(n)
{VERBOSE_REPORT;
  n =mat.n;
  array =mat.array;
  darray =mat.darray;
  
  mat.nullify();
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! dsymatrix destructor */
inline _dsymatrix::~_dsymatrix()
{VERBOSE_REPORT;
  delete[] array;
  delete[] darray;
}
