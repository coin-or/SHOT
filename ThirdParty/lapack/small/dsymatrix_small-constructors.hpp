//=============================================================================
/*!  */
template<long n>
inline dsymatrix_small<n>::dsymatrix_small()
{VERBOSE_REPORT;
  ;
}

//=============================================================================
/*!  */
template<long n>
inline dsymatrix_small<n>::dsymatrix_small(const dsymatrix& mat)
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
inline dsymatrix_small<n>::dsymatrix_small(const double& x)
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
inline dsymatrix_small<n>::~dsymatrix_small()
{VERBOSE_REPORT;
  ;
}
