//=============================================================================
/*! _zgbmatrix constructor */
inline _zgbmatrix::_zgbmatrix()
{VERBOSE_REPORT;
  m =0;
  n =0;
  kl =0;
  ku =0;
  array =NULL;
  darray =NULL;
}

//=============================================================================
/*! _zgbmatrix copy constructor */
inline _zgbmatrix::_zgbmatrix(const _zgbmatrix& mat)
{VERBOSE_REPORT;
  //////// initialize ////////
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
/*! _zgbmatrix destructor */
inline _zgbmatrix::~_zgbmatrix()
{VERBOSE_REPORT;
  delete [] array;
  delete [] darray;  
}
