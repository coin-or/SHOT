//=============================================================================
/*! zgematrix=zgematrix operator */
inline zgematrix& zgematrix::operator=(const zgematrix& mat)
{VERBOSE_REPORT;
  if(array!=mat.array){ // if it is NOT self substitution
    copy(mat);
  }
  return *this;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! zgematrix+=zgematrix operator */
inline zgematrix& zgematrix::operator+=(const zgematrix& mat)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(n!=mat.n || m!=mat.m){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a summation." << std::endl
              << "Your input was (" << m << "x" << n << ") += (" << mat.m << "x" << mat.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  for(long i=0; i<m*n; i++){ array[i]+=mat.array[i]; }
  return *this;
}

//=============================================================================
/*! zgematrix operator-= */
inline zgematrix& zgematrix::operator-=(const zgematrix& mat)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(n!=mat.n || m!=mat.m){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a sutraction." << std::endl
              << "Your input was (" << m << "x" << n << ") -= (" << mat.m << "x" << mat.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  for(long i=0; i<m*n; i++){ array[i]-=mat.array[i]; }
  return *this;
}

//=============================================================================
/*! zgematrix operator*= */
inline zgematrix& zgematrix::operator*=(const zgematrix& mat)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(n!=mat.m){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a product." << std::endl
              << "Your input was (" << m << "x" << n << ") *= (" << mat.m << "x" << mat.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  zgematrix newmat( m, mat.n );
  zgemm_( 'n', 'n', m, mat.n, n, comple(1.0,0.0), array, m,
          mat.array, mat.m, comple(0.0,0.0), newmat.array, m );

  swap(*this,newmat);
  return *this;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! zgematrix+zgematrix operator */
inline _zgematrix operator+(const zgematrix& matA, const zgematrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.n || matA.m!=matB.m){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a summation." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") + (" << matB.m << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG

  zgematrix newmat(matA.m,matA.n);
  for(long i=0; i<newmat.m*newmat.n; i++){
    newmat.array[i] =matA.array[i]+matB.array[i];
  }
  
  return _(newmat);
}

//=============================================================================
/*! zgematrix-zgematrix operator */
inline _zgematrix operator-(const zgematrix& matA, const zgematrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.n || matA.m!=matB.m){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a subtraction." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") - (" << matB.m << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG

  zgematrix newmat(matA.m,matA.n);
  for(long i=0; i<newmat.m*newmat.n; i++){
    newmat.array[i] =matA.array[i]-matB.array[i];
  }
  
  return _(newmat);
}

//=============================================================================
/*! zgematrix*zgematrix operator */
inline _zgematrix operator*(const zgematrix& matA, const zgematrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.m){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a product." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") * (" << matB.m << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  zgematrix newmat( matA.m, matB.n );
  zgemm_( 'n', 'n', matA.m, matB.n, matA.n, comple(1.0,0.0), 
          matA.array, matA.m, matB.array, matB.m, 
          comple(0.0,0.0), newmat.array, matA.m );
  
  return _(newmat);
}
