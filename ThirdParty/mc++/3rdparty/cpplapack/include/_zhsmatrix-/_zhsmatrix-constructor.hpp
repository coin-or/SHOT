//=============================================================================
/*! _zhsmatrix constructor without arguments */
inline _zhsmatrix::_zhsmatrix()
  :m(n)
{VERBOSE_REPORT;
  //////// initialize ////////
  n =0;
  data.clear();
  line.clear();
}

//=============================================================================
/*! _zhsmatrix copy constructor */
inline _zhsmatrix::_zhsmatrix(const _zhsmatrix& mat)
  :m(n)
{VERBOSE_REPORT;
  //////// initialize ////////
  n =mat.n;
  data.swap(mat.data);
  line.swap(mat.line);
  
  mat.nullify();
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! _zhsmatrix destructor */
inline _zhsmatrix::~_zhsmatrix()
{VERBOSE_REPORT;
  data.clear();
  line.clear();
}
