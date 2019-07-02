//=============================================================================
/*!  */
template<long m, long n>
inline zgematrix_small<m,n>::zgematrix_small()
{VERBOSE_REPORT;
  ;
}

//=============================================================================
/*!  */
template<long m, long n>
inline zgematrix_small<m,n>::zgematrix_small(const zgematrix& mat)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if( m!=mat.m || n!=mat.n ){
    ERROR_REPORT;
    std::cerr << "Matrix sizes must be the same." << std::endl
              << "Your input was " << m << "x" << n << " and " << mat.m << "x" << mat.n << "." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  for(long k=0; k<m*n; k++){
    array[k] =mat.array[k];
  }
}

//=============================================================================
/*!  */
template<long m, long n>
inline zgematrix_small<m,n>::zgematrix_small(const comple& x)
{VERBOSE_REPORT;
  for(long k=0; k<m*n; k++){
    array[k] =x;
  }
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*!  */
template<long m, long n>
inline zgematrix_small<m,n>::~zgematrix_small()
{VERBOSE_REPORT;
  ;
}
