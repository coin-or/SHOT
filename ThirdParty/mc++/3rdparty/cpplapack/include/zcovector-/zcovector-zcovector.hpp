//=============================================================================
/*! zcovector=zcovector operator */
inline zcovector& zcovector::operator=(const zcovector& vec)
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
/*! zcovector+=zcovector operator */
inline zcovector& zcovector::operator+=(const zcovector& vec)
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
/*! zcovector operator-= */
inline zcovector& zcovector::operator-=(const zcovector& vec)
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
/*! zcovector+zcovector operator */
inline _zcovector operator+(const zcovector& vecA, const zcovector& vecB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(vecA.l!=vecB.l){
    ERROR_REPORT;
    std::cerr << "These two vectors can not make a sumation." << std::endl
              << "Your input was (" << vecA.l << ") + (" << vecB.l << ")." << std::endl;
    exit(1);
  }
  
#endif//CPPL_DEBUG
  
  zcovector newvec(vecA.l);
  for(long i=0; i<newvec.l; i++){
    newvec.array[i] =vecA.array[i]+vecB.array[i];
  }
  
  return _(newvec);
}

//=============================================================================
/*! zcovector-zcovector operator */
inline _zcovector operator-(const zcovector& vecA, const zcovector& vecB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(vecA.l!=vecB.l){
    ERROR_REPORT;
    std::cerr << "These two vectors can not make a subtraction." << std::endl
              << "Your input was (" << vecA.l << ") - (" << vecB.l << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  zcovector newvec(vecA.l);
  for(long i=0; i<newvec.l; i++){
    newvec.array[i] =vecA.array[i]-vecB.array[i];
  }
  
  return _(newvec);
}

//=============================================================================
/*! zcovector^T*zcovector operator (inner product) */
inline comple operator%(const zcovector& vecA, const zcovector& vecB)
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
  
  return val;
}
