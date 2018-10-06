//=============================================================================
/*!  */
template<long m, long n>
inline _dgematrix dgematrix_small<m,n>::to_dgematrix() const
{VERBOSE_REPORT;
  dgematrix mat(m,n);
  for(long i=0; i<m; i++){
    for(long j=0; j<n; j++){
      mat(i,j) =(*this)(i,j);
    }
  }
  return _(mat);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*!  */
template<long m, long n>
inline double& dgematrix_small<m,n>::operator()(const long& i, const long& j)
{VERBOSE_REPORT;
  return array[i+m*j];
}

//=============================================================================
/*!  */
template<long m, long n>
inline double dgematrix_small<m,n>::operator()(const long& i, const long& j) const
{VERBOSE_REPORT;
  return array[i+m*j];
}

//=============================================================================
/*!  */
template<long m, long n>
inline dgematrix_small<m,n>& dgematrix_small<m,n>::set(const long& i, const long& j, const double& v)
{VERBOSE_REPORT;
  (*this)(i,j) =v;
  return *this;
}

//=============================================================================
/*!  */
template<long m, long n>
inline std::ostream& operator<<(std::ostream& s, const dgematrix_small<m,n>& A)
{VERBOSE_REPORT;
  s << std::setiosflags(std::ios::showpos);
  for(long i=0; i<m; i++){
    for(long j=0; j<n; j++){
      s << " " << A(i,j);
    }
    s << std::endl;
  }
  return s;
}

//=============================================================================
/*!  */
template<long m, long n>
inline void dgematrix_small<m,n>::write(const char* filename) const
{VERBOSE_REPORT;
  std::ofstream ofs(filename, std::ios::trunc);
  ofs.setf(std::cout.flags());
  ofs.precision(std::cout.precision());
  ofs.width(std::cout.width());
  ofs.fill(std::cout.fill());
  ofs << "#dgematrix" << " " << m << " " << n << std::endl;
  for(long i=0; i<m; i++){
    for(long j=0; j<n; j++){
      ofs << (*this)()(i,j) << " ";
    }
    ofs << std::endl;
  }
  ofs.close();
}

//=============================================================================
/*!  */
template<long m, long n>
inline void dgematrix_small<m,n>::read(const char* filename)
{VERBOSE_REPORT;
  std::ifstream s( filename );
  if(!s){
    ERROR_REPORT;
    std::cerr << "The file \"" << filename << "\" can not be opened." << std::endl;
    exit(1);
  }
  
  std::string id;
  s >> id;
  if( id != "dgematrix" && id != "#dgematrix" ){
    ERROR_REPORT;
    std::cerr << "The type name of the file \"" << filename << "\" is not dgematrix." << std::endl
              << "Its type name was " << id << " ." << std::endl;
    exit(1);
  }
  
  long _m, _n;
  s >> _m >> _n;
  if(m!=_m || n!=_n){
    ERROR_REPORT;
    std::cerr << "Matrix size is invalid." << std::endl;
    exit(1);
  }
  for(long i=0; i<m; i++){
    for(long j=0; j<n; j++ ){
      s >> operator()(i,j);
    }
  }
  if(s.eof()){
    ERROR_REPORT;
    std::cerr << "There is something is wrong with the file \"" << filename << "\"." << std::endl
              << "Most likely, there is not enough data components, or a linefeed code or space code is missing at the end of the last line." << std::endl;
    exit(1);
  }
  
  s >> id;
  if(!s.eof()){
    ERROR_REPORT;
    std::cerr << "There is something is wrong with the file \"" << filename << "\"." << std::endl
              << "Most likely, there are extra data components." << std::endl;
    exit(1);
  }
  
  s.close();    
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*!  */
template<long m, long n>
inline dgematrix_small<n,m> t(const dgematrix_small<m,n>& A)
{VERBOSE_REPORT;
  dgematrix_small<n,m> X;
  for(long i=0; i<m; i++){
    for(long j=0; j<n; j++){
      X(j,i) =A(i,j);
    }
  }
  return X;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*!  */
template<long m, long n>
inline dgematrix_small<m,n>& dgematrix_small<m,n>::zero()
{VERBOSE_REPORT;
  for(long k=0; k<m*n; k++){ array[k]=0.; }
  return *this;
}

//=============================================================================
/*!  */
template<long m, long n>
inline dgematrix_small<m,n>& dgematrix_small<m,n>::identity()
{VERBOSE_REPORT;
  zero();
  for(long k=0; k<std::min(m,n); k++){ (*this)(k,k)=1.; }
  return *this;
}

//=============================================================================
/*!  */
template<long m, long n>
inline dcovector_small<m> dgematrix_small<m,n>::col(const long& j) const
{VERBOSE_REPORT;
  dcovector_small<m> vec;
  for(long i=0; i<m; i++){ vec(i)=(*this)(i,j); }
  return vec;
}

//=============================================================================
/*!  */
template<long m, long n>
inline drovector_small<n> dgematrix_small<m,n>::row(const long& i) const
{VERBOSE_REPORT;
  drovector_small<n> vec;
  for(long j=0; j<n; j++){ vec(j)=(*this)(i,j); }
  return vec;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*!  */
template<long m, long n>
inline dgematrix_small<m,n>& operator+=(dgematrix_small<m,n>& A, const dgematrix_small<m,n>& B)
{VERBOSE_REPORT;
  for(long k=0; k<m*n; k++){
    A.array[k] +=B.array[k];
  }
  return A;
}

//=============================================================================
/*!  */
template<long m, long n>
inline dgematrix_small<m,n>& operator-=(dgematrix_small<m,n>& A, const dgematrix_small<m,n>& B)
{VERBOSE_REPORT;
  for(long k=0; k<m*n; k++){
    A.array[k] -=B.array[k];
  }
  return A;
}

//=============================================================================
/*!  */
template<long m, long l, long n>
inline dgematrix_small<m,n>& operator*=(dgematrix_small<m,l>& A, const dgematrix_small<l,n>& B)
{VERBOSE_REPORT;
  dgematrix_small<m,n> X(0.);
  for(long i=0; i<m; i++){
    for(long j=0; j<n; j++){
      for(long k=0; k<l; k++){
        X(i,j) += A(i,k)*B(k,j);
      }
    }
  }
  return X;
}

//=============================================================================
/*!  */
template<long m, long n>
inline dgematrix_small<m,n>& operator*=(dgematrix_small<m,n>& A, const double& d)
{VERBOSE_REPORT;
  for(long k=0; k<m*n; k++){
    A.array[k] *=d;
  }
  return A;
}

//=============================================================================
/*!  */
template<long m, long n>
inline dgematrix_small<m,n>& operator/=(dgematrix_small<m,n>& A, const double& d)
{VERBOSE_REPORT;
  for(long k=0; k<m*n; k++){
    A.array[k] /=d;
  }
  return A;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! unary */
template<long m, long n>
inline const dgematrix_small<m,n>& operator+(const dgematrix_small<m,n>& A)
{VERBOSE_REPORT;
  return A;
}

//=============================================================================
/*! unary */
template<long m, long n>
inline dgematrix_small<m,n> operator-(const dgematrix_small<m,n>& A)
{VERBOSE_REPORT;
  dgematrix_small<m,n> X;
  for(long i=0; i<m; i++){
    for(long j=0; j<n; j++){
      X(i,j) =-A(i,j);
    }
  }
  return X;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*!  */
template<long m, long n>
inline dgematrix_small<m,n> operator+(const dgematrix_small<m,n>& A, const dgematrix_small<m,n>& B)
{VERBOSE_REPORT;
  dgematrix_small<m,n> C;
  for(int i=0; i<m; i++){
    for(int j=0; j<n; j++){
      C(i,j) =A(i,j)+B(i,j);
    }
  }
  return C;
}

//=============================================================================
/*!  */
template<long n>
inline dgematrix_small<n,n> operator+(const dgematrix_small<n,n>& A, const dsymatrix_small<n>& B)
{VERBOSE_REPORT;
  dgematrix_small<n,n> X;
  for(long i=0; i<n; i++){
    for(long j=0; j<i; j++){
      X(i,j) =A(i,j)+B(i,j);
    }
    for(long j=i; j<n; j++){
      X(i,j) =A(i,j)+B(j,i);
    }
  }
  return X;
}

//=============================================================================
/*!  */
template<long m, long n>
inline dgematrix_small<m,n> operator-(const dgematrix_small<m,n>& A, const dgematrix_small<m,n>& B)
{VERBOSE_REPORT;
  dgematrix_small<m,n> C;
  for(int i=0; i<m; i++){
    for(int j=0; j<n; j++){
      C(i,j)=A(i,j)-B(i,j);
    }
  }
  return C;
}

//=============================================================================
/*!  */
template<long n>
inline dgematrix_small<n,n> operator-(const dgematrix_small<n,n>& A, const dsymatrix_small<n>& B)
{VERBOSE_REPORT;
  dgematrix_small<n,n> X;
  for(long i=0; i<n; i++){
    for(long j=0;   j<=i; j++){ X(i,j)=A(i,j)-B(i,j); }
    for(long j=i+1; j<n;  j++){ X(i,j)=A(i,j)-B(j,i); }
  }
  return X;
}

//=============================================================================
/*!  */
template<long m, long n>
inline dcovector_small<m> operator*(const dgematrix_small<m,n>& A, const dcovector_small<n>& B)
{VERBOSE_REPORT;
  dcovector_small<m> C(0.);
  for(long i=0; i<m; i++){
    for(long j=0; j<n; j++){
      C(i) +=A(i,j)*B(j);
    }
  }
  return C;
}

//=============================================================================
/*!  */
template<long m, long l, long n>
inline dgematrix_small<m,n> operator*(const dgematrix_small<m,l>& A, const dgematrix_small<l,n>& B)
{VERBOSE_REPORT;
  dgematrix_small<m,n> C(0.);
  for(int i=0; i<m; i++){
    for(int j=0; j<n; j++){
      for(int k=0; k<l; k++){
        C(i,j) +=A(i,k)*B(k,j);
      }
    }
  }
  return C;
}

//=============================================================================
/*!  */
template<long m, long n>
inline dgematrix_small<m,n> operator*(const dgematrix_small<m,n>& A, const dsymatrix_small<n>& B)
{VERBOSE_REPORT;
  dgematrix_small<m,n> X(0.);
  for(long i=0; i<m; i++){
    for(long j=0; j<n; j++){
      for(long k=0; k<j; k++){ X(i,j)+=A(i,k)*B(j,k); }
      for(long k=j; k<n; k++){ X(i,j)+=A(i,k)*B(k,j); }
    }
  }
  return X;
}

//=============================================================================
/*!  */
template<long m, long n>
inline dgematrix_small<m,n> operator*(const dgematrix_small<m,n>& A, const double& v)
{VERBOSE_REPORT;
  dgematrix_small<m,n> C;
  for(long i=0; i<m; i++){
    for(long j=0; j<n; j++){
      C(i,j) =A(i,j)*v;
    }
  }
  return C;
}

//=============================================================================
/*!  */
template<long m, long n>
inline dgematrix_small<m,n> operator/(const dgematrix_small<m,n>& A, const double& v)
{VERBOSE_REPORT;
  dgematrix_small<m,n> C;
  for(long i=0; i<m; i++){
    for(long j=0; j<n; j++){
      C(i,j) =A(i,j)/v;
    }
  }
  return C;
}
