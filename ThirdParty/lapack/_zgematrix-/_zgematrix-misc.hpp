//=============================================================================
/*! nullify all the matrix data */
inline void _zgematrix::nullify() const
{VERBOSE_REPORT;
  m=0;
  n=0;
  array=NULL;
  darray=NULL;
}

//=============================================================================
/*! destroy all the matrix data */
inline void _zgematrix::destroy() const
{VERBOSE_REPORT;
  delete [] array;
  delete [] darray;
  array=NULL;
  darray=NULL;
}
