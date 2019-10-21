//=============================================================================
/*! dssmatrix constructor without arguments */
inline dssmatrix::dssmatrix()
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
/*! dssmatrix copy constructor */
inline dssmatrix::dssmatrix(const dssmatrix& mat)
  : m(n)
{VERBOSE_REPORT;
  data.clear();
  line.clear();
  copy(mat);
}

//=============================================================================
/*! dssmatrix constructor to cast _dssmatrix */
inline dssmatrix::dssmatrix(const _dssmatrix& mat)
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
/*! dssmatrix constructor with size specification */
inline dssmatrix::dssmatrix(const long& _n, const long _c)
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
/*! dssmatrix constructor with filename */
inline dssmatrix::dssmatrix(const char* filename)
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
/*! dssmatrix destructor */
inline dssmatrix::~dssmatrix()
{VERBOSE_REPORT;
  data.clear();
  line.clear();
}
