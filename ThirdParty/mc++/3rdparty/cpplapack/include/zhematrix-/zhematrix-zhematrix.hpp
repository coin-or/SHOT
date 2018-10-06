//=============================================================================
/*! zhematrix=zhematrix operator */
inline zhematrix& zhematrix::operator=(const zhematrix& mat)
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
/*! zhematrix+=zhematrix operator */
inline zhematrix& zhematrix::operator+=(const zhematrix& mat)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(n!=mat.n){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a summation." << std::endl
              << "Your input was (" << n << "x" << n << ") += (" << mat.n << "x" << mat.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  for(long i=0; i<n; i++){ for(long j=0; j<=i; j++){
    operator()(i,j) +=mat(i,j);
  }}
  
  return *this;
}

//=============================================================================
/*! zhematrix operator-= */
inline zhematrix& zhematrix::operator-=(const zhematrix& mat)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(n!=mat.n){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a sutraction." << std::endl
              << "Your input was (" << n << "x" << n << ") -= (" << mat.n << "x" << mat.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  for(long i=0; i<n; i++){ for(long j=0; j<=i; j++){
    operator()(i,j) -=mat(i,j);
  }}
  
  return *this;
}

//=============================================================================
/*! zhematrix+zhematrix operator */
inline _zhematrix operator+(const zhematrix& matA, const zhematrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.n){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a summation." << std::endl
              << "Your input was (" << matA.n << "x" << matA.n << ") + (" << matB.n << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG

  long n = matA.n;
  zhematrix newmat(n);
  for(long i=0; i<n; i++){ for(long j=0; j<=i; j++){
    newmat.array[i+n*j] = matA.array[i+n*j] + matB.array[i+n*j];
  }}
  
  return _(newmat);
}

//=============================================================================
/*! zhematrix-zhematrix operator */
inline _zhematrix operator-(const zhematrix& matA, const zhematrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.n){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a subtraction." << std::endl
              << "Your input was (" << matA.n << "x" << matA.n << ") - (" << matB.n << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG

  long n = matA.n;
  zhematrix newmat(n);
  for(long i=0; i<n; i++){ for(long j=0; j<=i; j++){
    newmat.array[i+n*j] =matA.array[i+n*j]-matB.array[i+n*j];
  }}
  
  return _(newmat);
}

//=============================================================================
/*! zhematrix*zhematrix operator */
inline _zgematrix operator*(const zhematrix& matA, const zhematrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.n){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a product." << std::endl
              << "Your input was (" << matA.n << "x" << matA.n << ") * (" << matB.n << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  matB.complete();
  zgematrix newmat( matA.n, matB.n );
  
  zhemm_( 'l', 'l', matA.n, matB.n, comple(1.0,0.0), 
          matA.array, matA.n, matB.array, matB.n, 
          comple(0.0,0.0), newmat.array, newmat.m );
  
  return _(newmat);
}
