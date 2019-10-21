//=============================================================================
/*! dgematrix constructor without arguments */
inline dgematrix::dgematrix()
{VERBOSE_REPORT;
  //////// initialize ////////
  m =0;
  n =0;
  array =NULL;
  darray =NULL;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! dgematrix copy constructor */
inline dgematrix::dgematrix(const dgematrix& mat)
{VERBOSE_REPORT;
  //////// initialize ////////
  m =mat.m;
  n =mat.n;
  array =new double[m*n];
  darray =new double*[n];
  for(int i=0; i<n; i++){ darray[i] =&array[i*m]; }
  
  //////// copy ////////
  dcopy_(m*n, mat.array, 1, array, 1);
}

//=============================================================================
/*! dgematrix constructor to cast _dgematrix */
inline dgematrix::dgematrix(const _dgematrix& mat)
{VERBOSE_REPORT;
  m =mat.m;
  n =mat.n;
  array =mat.array;
  darray =mat.darray;
  
  mat.nullify();
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! dgematrix constructor with size specification */
inline dgematrix::dgematrix(const long& _m, const long& _n)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if( _m<0 || _n<0 ){
    ERROR_REPORT;
    std::cerr << "Matrix sizes must be positive integers. " << std::endl
              << "Your input was (" << _m << "," << _n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  //////// initialize ////////
  m =_m;
  n =_n;
  array =new double[m*n];
  darray  =new double*[n];
  for(int i=0; i<n; i++){ darray[i] =&array[i*m]; }
}

//=============================================================================
/*! dgematrix constructor with filename */
inline dgematrix::dgematrix(const char* filename)
{VERBOSE_REPORT;
  array =NULL;
  darray =NULL;
  
  //// read ////
  read(filename);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! dgematrix destructor */
inline dgematrix::~dgematrix()
{VERBOSE_REPORT;
  delete [] darray;
  delete [] array;
}
