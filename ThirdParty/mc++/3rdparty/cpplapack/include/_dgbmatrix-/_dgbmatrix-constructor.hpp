//=============================================================================
/*! _dgbmatrix constructor */
inline _dgbmatrix::_dgbmatrix()
{VERBOSE_REPORT;
  m =0;
  n =0;
  kl =0;
  ku =0;
  array =NULL;
  darray =NULL;
}

//=============================================================================
/*! _dgbmatrix copy constructor */
inline _dgbmatrix::_dgbmatrix(const _dgbmatrix& mat)
{VERBOSE_REPORT;
  m =mat.m;
  n =mat.n;
  kl =mat.kl;
  ku =mat.ku;
  array =mat.array;
  darray =mat.darray;
  
  mat.nullify();
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! _dgbmatrix destructor */
inline _dgbmatrix::~_dgbmatrix()
{VERBOSE_REPORT;
  delete[] array;
  delete[] darray;
}
