//=============================================================================
/*! dgematrix=_dgematrix operator */
inline dgematrix& dgematrix::operator=(const _dgematrix& mat)
{VERBOSE_REPORT;
  shallow_copy(mat);
  return *this;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! dgematrix+=_dgematrix operator */
inline dgematrix& dgematrix::operator+=(const _dgematrix& mat)
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
  
  mat.destroy();
  return *this;
}

//=============================================================================
/*! dgematrix-=_dgematrix operator */
inline dgematrix& dgematrix::operator-=(const _dgematrix& mat)
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

  mat.destroy();
  return *this;
}

//=============================================================================
/*! dgematrix*=_dgematrix operator */
inline dgematrix& dgematrix::operator*=(const _dgematrix& mat)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(n!=mat.m){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a product." << std::endl
              << "Your input was (" << m << "x" << n << ") *= (" << mat.m << "x" << mat.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  dgematrix newmat( m, mat.n );
  dgemm_( 'n', 'n', m, mat.n, n, 1.0, array, m,
          mat.array, mat.m, 0.0, newmat.array, m );
  
  swap(*this,newmat);
  mat.destroy();
  return *this;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! dgematrix+_dgematrix operator */
inline _dgematrix operator+(const dgematrix& matA, const _dgematrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.n || matA.m!=matB.m){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a summation." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") + (" << matB.m << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG

  for(long i=0; i<matA.m*matA.n; i++){ matB.array[i] +=matA.array[i]; }
  
  return matB;
}

//=============================================================================
/*! dgematrix-_dgematrix operator */
inline _dgematrix operator-(const dgematrix& matA, const _dgematrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.n || matA.m!=matB.m){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a subtraction." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") - (" << matB.m << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG

  for(long i=0; i<matA.m*matA.n; i++){
    matB.array[i] =matA.array[i]-matB.array[i];
  }
  
  return matB;
}

//=============================================================================
/*! dgematrix*_dgematrix operator */
inline _dgematrix operator*(const dgematrix& matA, const _dgematrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.m){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a product." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") * (" << matB.m << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  dgematrix newmat( matA.m, matB.n );
  dgemm_( 'n', 'n', matA.m, matB.n, matA.n, 1.0, matA.array, matA.m,
          matB.array, matB.m, 0.0, newmat.array, matA.m );
  
  matB.destroy();
  return _(newmat);
}
