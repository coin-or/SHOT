//=============================================================================
/*! _drovector+_drovector operator */
inline _drovector operator+(const _drovector& vecA, const _drovector& vecB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(vecA.l!=vecB.l){
    ERROR_REPORT;
    std::cerr << "These two vectors can not make a sumation." << std::endl
              << "Your input was (" << vecA.l << ") + (" << vecB.l << ")." << std::endl;
    exit(1);
  }
  
#endif//CPPL_DEBUG
  
  for(long i=0; i<vecA.l; i++){ vecA.array[i]+=vecB.array[i]; }
  
  vecB.destroy();
  return vecA;
}

//=============================================================================
/*! _drovector-_drovector operator */
inline _drovector operator-(const _drovector& vecA, const _drovector& vecB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(vecA.l!=vecB.l){
    ERROR_REPORT;
    std::cerr << "These two vectors can not make a subtraction." << std::endl
              << "Your input was (" << vecA.l << ") - (" << vecB.l << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  for(long i=0; i<vecA.l; i++){ vecA.array[i]-=vecB.array[i]; }
  
  vecB.destroy();
  return vecA;
}

//=============================================================================
/*! _drovector^T*_drovector operator (inner product) */
inline double operator%(const _drovector& vecA, const _drovector& vecB)
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
  
  vecA.destroy();
  vecB.destroy();
  return val;
}
