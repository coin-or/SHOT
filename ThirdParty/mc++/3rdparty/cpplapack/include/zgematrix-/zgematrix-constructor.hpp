//=============================================================================
/*! zgematrix constructor without arguments */
inline zgematrix::zgematrix()
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
/*! zgematrix copy constructor */
inline zgematrix::zgematrix(const zgematrix& mat)
{VERBOSE_REPORT;
  //////// initialize ////////
  m =mat.m;
  n =mat.n;
  array =new comple[m*n];
  darray =new comple*[n];
  for(int i=0; i<n; i++){ darray[i] =&array[i*m]; }

  //////// copy ////////
  zcopy_(m*n, mat.array, 1, array, 1);
}

//=============================================================================
/*! zgematrix constructor to cast _zgematrix */
inline zgematrix::zgematrix(const _zgematrix& mat)
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
/*! zgematrix constructor with size specification */
inline zgematrix::zgematrix(const long& _m, const long& _n)
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
  array =new comple[m*n];
  darray =new comple*[n];
  for(int i=0; i<n; i++){ darray[i] =&array[i*m]; }
}

//=============================================================================
/*! zgematrix constructor with filename */
inline zgematrix::zgematrix(const char* filename)
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
/*! zgematrix destructor */
inline zgematrix::~zgematrix()
{VERBOSE_REPORT;
  //////// delete array ////////
  delete [] array;
  delete [] darray;
}
