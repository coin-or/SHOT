//=============================================================================
/*! dcovector=dcovector operator */
inline dcovector& dcovector::operator=(const dcovector& vec)
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
/*! dcovector+=dcovector operator */
inline dcovector& dcovector::operator+=(const dcovector& vec)
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
/*! dcovector operator-= */
inline dcovector& dcovector::operator-=(const dcovector& vec)
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
/*! dcovector+dcovector operator */
inline _dcovector operator+(const dcovector& vecA, const dcovector& vecB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(vecA.l!=vecB.l){
    ERROR_REPORT;
    std::cerr << "These two vectors can not make a sumation." << std::endl
              << "Your input was (" << vecA.l << ") + (" << vecB.l << ")." << std::endl;
    exit(1);
  }
  
#endif//CPPL_DEBUG
  
  dcovector newvec(vecA.l);
  for(long i=0; i<newvec.l; i++){
    newvec.array[i] =vecA.array[i]+vecB.array[i];
  }
  
  return _(newvec);
}

//=============================================================================
/*! dcovector-dcovector operator */
inline _dcovector operator-(const dcovector& vecA, const dcovector& vecB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(vecA.l!=vecB.l){
    ERROR_REPORT;
    std::cerr << "These two vectors can not make a subtraction." << std::endl
              << "Your input was (" << vecA.l << ") - (" << vecB.l << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  dcovector newvec(vecA.l);
  for(long i=0; i<newvec.l; i++){
    newvec.array[i] =vecA.array[i]-vecB.array[i];
  }
  
  return _(newvec);
}

//=============================================================================
/*! dcovector^T*dcovector operator (inner product) */
inline double operator%(const dcovector& vecA, const dcovector& vecB)
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
