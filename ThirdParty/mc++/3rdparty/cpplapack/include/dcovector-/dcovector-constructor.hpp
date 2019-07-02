//=============================================================================
/*! dcovector constructor */
inline dcovector::dcovector()
{VERBOSE_REPORT;
  //////// initialize ////////
  l =0;
  cap =0;
  array =NULL;
}

//=============================================================================
/*! dcovector copy constructor */
inline dcovector::dcovector(const dcovector& vec)
{VERBOSE_REPORT;
  //////// initialize ////////
  l =vec.l;
  cap =vec.cap;
  array =new double[cap];
  
  //////// copy ////////
  dcopy_(l, vec.array, 1, array, 1);
}

//=============================================================================
/*! dcovector constructor to cast _dcovector */
inline dcovector::dcovector(const _dcovector& vec)
{VERBOSE_REPORT;
  //////// initialize ////////
  l =vec.l;
  cap =vec.cap;
  array =vec.array;
  
  vec.nullify();
}

//=============================================================================
/*! dcovector constructor with size specification */
inline dcovector::dcovector(const long& _l, const long margin)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if( _l<0 ){
    ERROR_REPORT;
    std::cerr << "Vector size must be positive integers. " << std::endl
              << "Your input was (" << _l << ")." << std::endl;
    exit(1);
  }
  if( margin<0 ){
    ERROR_REPORT;
    std::cerr << "Vector margin must be zero or above. " << std::endl
              << "Your input was (" << _l << ", " << margin << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  //////// initialize ////////
  l =_l;
  cap =l+margin;
  array =new double[cap];
}

//=============================================================================
/*! dcovector constructor with filename */
inline dcovector::dcovector(const char* filename)
{VERBOSE_REPORT;
  array =NULL;
  read(filename);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! dcovector destructor */
inline dcovector::~dcovector()
{VERBOSE_REPORT;
  //////// delete array ////////
  delete [] array;
}
