//=============================================================================
/*! drovector constructor */
inline drovector::drovector()
{VERBOSE_REPORT;
  //////// initialize ////////
  l =0;
  cap =0;
  array =NULL;
}

//=============================================================================
/*! drovector copy constructor */
inline drovector::drovector(const drovector& vec)
{VERBOSE_REPORT;
  //////// initialize ////////
  l =vec.l;
  cap =vec.cap;
  array =new double[cap];
  
  //////// copy ////////
  dcopy_(l, vec.array, 1, array, 1);
}

//=============================================================================
/*! drovector constructor to cast _drovector */
inline drovector::drovector(const _drovector& vec)
{VERBOSE_REPORT;
  //////// initialize ////////
  l =vec.l;
  cap =vec.cap;
  array =vec.array;
  
  vec.nullify();
}

//=============================================================================
/*! drovector constructor with size specification */
inline drovector::drovector(const long& _l, const long margin)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if( _l<0 || margin<0 ){
    ERROR_REPORT;
    std::cerr << "Vector size must be positive integers. " << std::endl
              << "Your input was (" << _l << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  //////// initialize ////////
  l =_l;
  cap =l+margin;
  array =new double[cap];
}

//=============================================================================
/*! drovector constructor with filename */
inline drovector::drovector(const char* filename)
{VERBOSE_REPORT;
  array =NULL;
  read(filename);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! drovector destructor */
inline drovector::~drovector()
{VERBOSE_REPORT;
  delete [] array;
}
