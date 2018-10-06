//=============================================================================
/*! zrovector=_zrovector operator */
inline zrovector& zrovector::operator=(const _zrovector& vec)
{VERBOSE_REPORT;
  shallow_copy(vec);
  return *this;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! zrovector+=_zrovector operator */
inline zrovector& zrovector::operator+=(const _zrovector& vec)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if( l!=vec.l ){
    ERROR_REPORT;
    std::cerr << "These two vectors can not make a sumation." << std::endl
              << "Your input was (" << l << ") += (" << vec.l << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  for(long i=0; i<l; i++){ array[i]+=vec.array[i]; }
  
  vec.destroy();
  return *this;
}

//=============================================================================
/*! zrovector operator-= */
inline zrovector& zrovector::operator-=(const _zrovector& vec)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if( l!=vec.l ){
    ERROR_REPORT;
    std::cerr << "These two vectors can not make a subtraction." << std::endl
              << "Your input was (" << l << ") -= (" << vec.l << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  for(long i=0; i<l; i++){ array[i]-=vec.array[i]; }
  
  vec.destroy();
  return *this;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! zrovector+zrovector operator */
inline _zrovector operator+(const zrovector& vecA, const _zrovector& vecB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(vecA.l!=vecB.l){
    ERROR_REPORT;
    std::cerr << "These two vectors can not make a sumation." << std::endl
              << "Your input was (" << vecA.l << ") + (" << vecB.l << ")." << std::endl;
    exit(1);
  }
  
#endif//CPPL_DEBUG
  
  for(long i=0; i<vecA.l; i++){ vecB.array[i]+=vecA.array[i]; }
  
  return vecB;
}

//=============================================================================
/*! zrovector-zrovector operator */
inline _zrovector operator-(const zrovector& vecA, const _zrovector& vecB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(vecA.l!=vecB.l){
    ERROR_REPORT;
    std::cerr << "These two vectors can not make a subtraction." << std::endl
              << "Your input was (" << vecA.l << ") - (" << vecB.l << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  for(long i=0; i<vecA.l; i++){
    vecB.array[i] =vecA.array[i]-vecB.array[i];
  }
  
  return vecB;
}

//=============================================================================
/*! zrovector^T*zrovector operator (inner product) */
inline comple operator%(const zrovector& vecA, const _zrovector& vecB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(vecA.l!=vecB.l){
    ERROR_REPORT;
    std::cerr << "These two vectors can not make a dot product." << std::endl
              << "Your input was (" << vecA.l << ") % (" << vecB.l << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  comple val( zdotu_( vecA.l, vecA.array, 1, vecB.array, 1 ) );
  
  vecB.destroy();
  return val;
}
