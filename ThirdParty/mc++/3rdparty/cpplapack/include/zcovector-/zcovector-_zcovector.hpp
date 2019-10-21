//=============================================================================
/*! zcovector=_zcovector operator */
inline zcovector& zcovector::operator=(const _zcovector& vec)
{VERBOSE_REPORT;
  shallow_copy(vec);
  return *this;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! zcovector+=_zcovector operator */
inline zcovector& zcovector::operator+=(const _zcovector& vec)
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
/*! zcovector operator-= */
inline zcovector& zcovector::operator-=(const _zcovector& vec)
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
/*! zcovector+zcovector operator */
inline _zcovector operator+(const zcovector& vecA, const _zcovector& vecB)
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
/*! zcovector-zcovector operator */
inline _zcovector operator-(const zcovector& vecA, const _zcovector& vecB)
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
/*! zcovector^T*zcovector operator (inner product) */
inline comple operator%(const zcovector& vecA, const _zcovector& vecB)
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
