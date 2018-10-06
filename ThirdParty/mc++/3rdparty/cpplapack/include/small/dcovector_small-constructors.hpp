//=============================================================================
/*!  */
template<long l>
inline dcovector_small<l>::dcovector_small()
{VERBOSE_REPORT;
  ;
}

//=============================================================================
/*!  */
template<long l>
inline dcovector_small<l>::dcovector_small(const dcovector& vec)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if( l!=vec.l ){
    ERROR_REPORT;
    std::cerr << "Vector sizes must be the same." << std::endl
              << "Your input was " << l << " and " << vec.l << "." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  for(long k=0; k<l; k++){
    array[k] =vec.array[k];
  }
}

//=============================================================================
/*!  */
template<long l>
inline dcovector_small<l>::dcovector_small(const double& x)
{VERBOSE_REPORT;
  for(long k=0; k<l; k++){
    array[k] =x;
  }
}

//=============================================================================
/*!  */
template<long l>
inline dcovector_small<l>::dcovector_small(const double& x, const double& y)
{VERBOSE_REPORT;
  array[0] =x;
  array[1] =y;
}

//=============================================================================
/*!  */
template<long l>
inline dcovector_small<l>::dcovector_small(const double& x, const double& y, const double& z)
{VERBOSE_REPORT;
  array[0] =x;
  array[1] =y;
  array[2] =z;
}

//=============================================================================
/*!  */
template<long l>
inline dcovector_small<l>::dcovector_small(const double& x, const double& y, const double& z, const double& r)
{VERBOSE_REPORT;
  array[0] =x;
  array[1] =y;
  array[2] =z;
  array[3] =r;
}
  
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*!  */
template<long l>
inline dcovector_small<l>::~dcovector_small()
{VERBOSE_REPORT;
  ;
}
