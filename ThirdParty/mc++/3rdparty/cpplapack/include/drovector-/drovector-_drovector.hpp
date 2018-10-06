//=============================================================================
/*! drovector=_drovector operator */
inline drovector& drovector::operator=(const _drovector& vec)
{VERBOSE_REPORT;
  shallow_copy(vec);
  return *this;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! drovector+=_drovector operator */
inline drovector& drovector::operator+=(const _drovector& vec)
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
/*! drovector operator-= */
inline drovector& drovector::operator-=(const _drovector& vec)
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
/*! drovector+drovector operator */
inline _drovector operator+(const drovector& vecA, const _drovector& vecB)
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
/*! drovector-drovector operator */
inline _drovector operator-(const drovector& vecA, const _drovector& vecB)
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
/*! drovector^T*drovector operator (inner product) */
inline double operator%(const drovector& vecA, const _drovector& vecB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(vecA.l!=vecB.l){
    ERROR_REPORT;
    std::cerr << "These two vectors can not make a dot product." << std::endl
              << "Your input was (" << vecA.l << ") % (" << vecB.l << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  double val( ddot_( vecA.l, vecA.array, 1, vecB.array, 1 ) );
  
  vecB.destroy();
  return val;
}
