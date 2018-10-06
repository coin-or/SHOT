//=============================================================================
/*!  */
template<long l>
inline dcovector_small<l> operator*(const double& v, const dcovector_small<l>& A)
{VERBOSE_REPORT;
  dcovector_small<l> X;
  for(long i=0; i<l; i++){
    X(i) =v*A(i);
  }
  return X;
}

//=============================================================================
/*!  */
template<long l>
inline drovector_small<l> operator*(const double& v, const drovector_small<l>& A)
{VERBOSE_REPORT;
  drovector_small<l> X;
  for(long i=0; i<l; i++){
    X(i) =v*A(i);
  }
  return X;
}

//=============================================================================
/*!  */
template<long m, long n>
inline dgematrix_small<m,n> operator*(const double& v, const dgematrix_small<m,n>& A)
{VERBOSE_REPORT;
  dgematrix_small<m,n> C;
  for(long i=0; i<m; i++){
    for(long j=0; j<n; j++){
      C(i,j) =v*A(i,j);
    }
  }
  return C;
}

//=============================================================================
/*!  */
template<long n>
inline dsymatrix_small<n> operator*(const double& v, const dsymatrix_small<n>& A)
{VERBOSE_REPORT;
  dsymatrix_small<n> X;
  for(long k=0; k<(n*(n+1))/2; k++){
    X.array[k] =v*A.array[k];
  }
  return X;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*!  */
template<long l>
inline zcovector_small<l> operator*(const double& v, const zcovector_small<l>& A)
{VERBOSE_REPORT;
  zcovector_small<l> X;
  for(long i=0; i<l; i++){
    X(i) =v*A(i);
  }
  return X;
}

//=============================================================================
/*!  */
template<long l>
inline zrovector_small<l> operator*(const double& v, const zrovector_small<l>& A)
{VERBOSE_REPORT;
  zrovector_small<l> X;
  for(long i=0; i<l; i++){
    X(i) =v*A(i);
  }
  return X;
}

//=============================================================================
/*!  */
template<long m, long n>
inline zgematrix_small<m,n> operator*(const double& v, const zgematrix_small<m,n>& A)
{VERBOSE_REPORT;
  zgematrix_small<m,n> C;
  for(long i=0; i<m; i++){
    for(long j=0; j<n; j++){
      C(i,j) =v*A(i,j);
    }
  }
  return C;
}

//=============================================================================
/*!  */
template<long n>
inline zhematrix_small<n> operator*(const double& v, const zhematrix_small<n>& A)
{VERBOSE_REPORT;
  zhematrix_small<n> X;
  for(long k=0; k<(n*(n+1))/2; k++){
    X.array[k] =v*A.array[k];
  }
  return X;
}
