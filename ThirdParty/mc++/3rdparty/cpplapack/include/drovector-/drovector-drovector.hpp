//=============================================================================
/*! drovector=drovector operator */
inline drovector& drovector::operator=(const drovector& vec)
{VERBOSE_REPORT;
  if(array!=vec.array){ // if it is NOT self substitution
    copy(vec);
  }
  return *this;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! drovector+=drovector operator */
inline drovector& drovector::operator+=(const drovector& vec)
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
  
  return *this;
}

//=============================================================================
/*! drovector operator-= */
inline drovector& drovector::operator-=(const drovector& vec)
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
  
  return *this;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! drovector+drovector operator */
inline _drovector operator+(const drovector& vecA, const drovector& vecB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(vecA.l!=vecB.l){
    ERROR_REPORT;
    std::cerr << "These two vectors can not make a sumation." << std::endl
              << "Your input was (" << vecA.l << ") + (" << vecB.l << ")." << std::endl;
    exit(1);
  }
  
#endif//CPPL_DEBUG
  
  drovector newvec(vecA.l);
  for(long i=0; i<newvec.l; i++){
    newvec.array[i] =vecA.array[i]+vecB.array[i];
  }
  
  return _(newvec);
}

//=============================================================================
/*! drovector-drovector operator */
inline _drovector operator-(const drovector& vecA, const drovector& vecB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(vecA.l!=vecB.l){
    ERROR_REPORT;
    std::cerr << "These two vectors can not make a subtraction." << std::endl
              << "Your input was (" << vecA.l << ") - (" << vecB.l << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  drovector newvec(vecA.l);
  for(long i=0; i<newvec.l; i++){
    newvec.array[i] =vecA.array[i]-vecB.array[i];
  }
  
  return _(newvec);
}

//=============================================================================
/*! drovector^T*drovector operator (inner product) */
inline double operator%(const drovector& vecA, const drovector& vecB)
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
  
  return val;
}
