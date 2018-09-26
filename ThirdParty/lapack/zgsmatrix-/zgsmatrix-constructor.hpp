//=============================================================================
/*! zgsmatrix constructor without arguments */
inline zgsmatrix::zgsmatrix()
{VERBOSE_REPORT;
  //////// initialize ////////
  m =0;
  n =0;
  data.clear();
  rows.clear();
  cols.clear();
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! zgsmatrix copy constructor */
inline zgsmatrix::zgsmatrix(const zgsmatrix& mat)
{VERBOSE_REPORT;
  m =mat.m;
  n =mat.n;
  data.clear();
  rows.clear();
  cols.clear();
  copy(mat);
}

//=============================================================================
/*! zgsmatrix constructor to cast _zgsmatrix */
inline zgsmatrix::zgsmatrix(const _zgsmatrix& mat)
{VERBOSE_REPORT;
  m =mat.m;
  n =mat.n;
  data.clear();
  rows.clear();
  cols.clear();
  
  data.swap(mat.data);
  rows.swap(mat.rows);
  cols.swap(mat.cols);
  
  mat.nullify();
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! zgsmatrix constructor with size specification */
inline zgsmatrix::zgsmatrix(const long& _m, const long& _n, const long _c)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if( _m<0 || _n<0 || _c<0 ){
    ERROR_REPORT;
    std::cerr << "Matrix sizes and the length of arrays must be positive integers. " << std::endl
              << "Your input was (" << _m << "," << _n << "," << _c << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  //////// initialize ////////
  m =_m;
  n =_n;
  data.resize(0);
  data.reserve(_c);
  rows.resize(m);
  cols.resize(n);
}

//=============================================================================
/*! zgsmatrix constructor with filename */
inline zgsmatrix::zgsmatrix(const char* filename)
{VERBOSE_REPORT;
  data.clear();
  rows.clear();
  cols.clear();
  
  //// read ////
  read(filename);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! zgsmatrix destructor */
inline zgsmatrix::~zgsmatrix()
{VERBOSE_REPORT;
  data.clear();
  rows.clear();
  cols.clear();
}
