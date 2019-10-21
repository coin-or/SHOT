//=============================================================================
/*! zhsmatrix constructor without arguments */
inline zhsmatrix::zhsmatrix()
  : m(n)
{VERBOSE_REPORT;
  //////// initialize ////////
  n =0;
  data.clear();
  line.clear();
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! zhsmatrix copy constructor */
inline zhsmatrix::zhsmatrix(const zhsmatrix& mat)
  : m(n)
{VERBOSE_REPORT;
  data.clear();
  line.clear();
  copy(mat);
}

//=============================================================================
/*! zhsmatrix constructor to cast _zhsmatrix */
inline zhsmatrix::zhsmatrix(const _zhsmatrix& mat)
  : m(n)
{VERBOSE_REPORT;
  n =mat.n;
  data.clear();
  line.clear();
  
  data.swap(mat.data);
  line.swap(mat.line);
  
  mat.nullify();
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! zhsmatrix constructor with size specification */
inline zhsmatrix::zhsmatrix(const long& _n, const long _c)
  : m(n)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if( _n<0 || _c<0 ){
    ERROR_REPORT;
    std::cerr << "Matrix sizes and the length of arrays must be positive integers. " << std::endl
              << "Your input was (" << _n << "," << _c << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  //////// initialize ////////
  n =_n;
  data.resize(0);
  data.reserve(_c);
  line.resize(n);
}

//=============================================================================
/*! zhsmatrix constructor with filename */
inline zhsmatrix::zhsmatrix(const char* filename)
  : m(n)
{VERBOSE_REPORT;
  data.clear();
  line.clear();
  
  //// read ////
  read(filename);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! zhsmatrix destructor */
inline zhsmatrix::~zhsmatrix()
{VERBOSE_REPORT;
  data.clear();
  line.clear();
}
