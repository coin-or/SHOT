//=============================================================================
/*! zgbmatrix constructor */
inline zgbmatrix::zgbmatrix()
{VERBOSE_REPORT;
  //////// initialize ////////
  m =0;
  n =0;
  kl =0;
  ku =0;
  array =NULL;
  darray =NULL;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! zgbmatrix copy constructor */
inline zgbmatrix::zgbmatrix(const zgbmatrix& mat)
{VERBOSE_REPORT;
  //////// initialize ////////
  m =mat.m;
  n =mat.n;
  kl =mat.kl;
  ku =mat.ku;
  array =new comple[(kl+ku+1)*n];
  darray =new comple*[n];
  for(int i=0; i<n; i++){ darray[i] =&array[i*(kl+ku+1)]; }

  //////// copy ////////
  zcopy_((kl+ku+1)*n, mat.array, 1, array, 1);
}

//=============================================================================
/*! zgbmatrix constructor to cast _zgbmatrix */
inline zgbmatrix::zgbmatrix(const _zgbmatrix& mat)
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
/*! zgbmatrix constructor with size specification */
inline zgbmatrix::zgbmatrix(const long& _m, const long& _n,
                            const long& _kl, const long& _ku)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if( _m<0 || _n<0 || _kl<0 || _ku<0 || _m<_kl || _n<_ku ){
    ERROR_REPORT;
    std::cerr << "It is impossible to make a matrix you ordered. " << std::endl
              << "Your input was (" << _m << "," << _n << ","<< _ku << "," << _kl << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  //////// initialize ////////
  m =_m;
  n =_n;
  kl =_kl;
  ku =_ku;
  array =new comple[(kl+ku+1)*n];
  darray =new comple*[n];
  for(int i=0; i<n; i++){ darray[i] =&array[i*(kl+ku+1)]; }
}

//=============================================================================
/*! zgbmatrix constructor with filename */
inline zgbmatrix::zgbmatrix(const char* filename)
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
/*! zgbmatrix destructor */
inline zgbmatrix::~zgbmatrix()
{VERBOSE_REPORT;
  //////// delete array ////////
  delete [] array;
  delete [] darray;
}
