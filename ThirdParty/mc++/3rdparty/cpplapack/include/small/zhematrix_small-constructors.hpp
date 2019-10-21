//=============================================================================
/*!  */
template<long n>
inline zhematrix_small<n>::zhematrix_small()
{VERBOSE_REPORT;
  ;
}

//=============================================================================
/*!  */
template<long n>
inline zhematrix_small<n>::zhematrix_small(const zhematrix& mat)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if( n!=mat.n ){
    ERROR_REPORT;
    std::cerr << "Matrix sizes must be the same." << std::endl
              << "Your input was " << n << "x" << n << " and " << mat.m << "x" << mat.n << "." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  for(long k=0; k<(n*(n+1))/2; k++){
    array[k] =mat.array[k];;
  }
}

//=============================================================================
/*!  */
template<long n>
inline zhematrix_small<n>::zhematrix_small(const comple& x)
{VERBOSE_REPORT;
  for(long k=0; k<(n*(n+1))/2; k++){
    array[k] =x;
  }
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*!  */
template<long n>
inline zhematrix_small<n>::~zhematrix_small()
{VERBOSE_REPORT;
  ;
}
