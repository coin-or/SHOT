//=============================================================================
/*!  */
template<long l>
inline zrovector_small<l>::zrovector_small()
{VERBOSE_REPORT;
  ;
}

//=============================================================================
/*!  */
template<long l>
inline zrovector_small<l>::zrovector_small(const zrovector& vec)
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
inline zrovector_small<l>::zrovector_small(const comple& x)
{VERBOSE_REPORT;
  for(long k=0; k<l; k++){
    array[k] =x;
  }
}

//=============================================================================
/*!  */
template<long l>
inline zrovector_small<l>::zrovector_small(const comple& x, const comple& y)
{VERBOSE_REPORT;
  array[0] =x;
  array[1] =y;
}

//=============================================================================
/*!  */
template<long l>
inline zrovector_small<l>::zrovector_small(const comple& x, const comple& y, const comple& z)
{VERBOSE_REPORT;
  array[0] =x;
  array[1] =y;
  array[2] =z;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*!  */
template<long l>
inline zrovector_small<l>::~zrovector_small()
{VERBOSE_REPORT;
  ;
}
