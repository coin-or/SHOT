//=============================================================================
/*!  */
template<long n>
inline zgematrix_small<n,n> zhematrix_small<n>::to_zgematrix_small() const
{VERBOSE_REPORT;
  zgematrix_small<n,n> newmat;
  for(long i=0; i<n; i++){
    for(long j=0;   j<=i; j++){ newmat(i,j) =(*this)(i,j); }
    for(long j=i+1; j<n;  j++){ newmat(i,j) =(*this)(j,i); }
  }
  return newmat;
}

//=============================================================================
/*!  */
template<long n>
inline zhematrix zhematrix_small<n>::to_zhematrix() const
{VERBOSE_REPORT;
  zhematrix newmat(n);
  for(long i=0; i<n; i++){
    for(long j=0; j<=i; j++){
      newmat(i,j) =(*this)(i,j);
    }
  }
  return newmat;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*!  */
template<long n>
inline comple& zhematrix_small<n>::operator()(const long& i, const long& j)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(i<j){
    ERROR_REPORT;
    std::cerr << "i<j" << std::endl;
    exit(1);
  }
#endif
  
  //const long I(max(i,j)), J(min(i,j)); return array[(I*(I+1))/2 +J];
  return array[(i*(i+1))/2 +j]; //l storage
}

//=============================================================================
/*!  */
template<long n>
inline comple zhematrix_small<n>::operator()(const long& i, const long& j) const
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(i<j){
    ERROR_REPORT;
    std::cerr << "i<j" << std::endl;
    exit(1);
  }
#endif
  
  //const long I(max(i,j)), J(min(i,j)); return array[(I*(I+1))/2 +J];
  return array[(i*(i+1))/2 +j];
}

//=============================================================================
/*!  */
template<long n>
inline zhematrix_small<n>& zhematrix_small<n>::set(const long& i, const long& j, const comple& v)
{VERBOSE_REPORT;
  (*this)(i,j)=v;
  return *this;
}

//=============================================================================
/*!  */
template<long n>
inline std::ostream& operator<<(std::ostream& s, const zhematrix_small<n>& A)
{VERBOSE_REPORT;
  s << std::setiosflags(std::ios::showpos);
  for(long i=0; i<n; i++){
    for(long j=0;   j<=i; j++){ s << " " << A(i,j) << " "<< std::flush; }
    for(long j=i+1; j<n;  j++){ s << "{" << A(j,i) << "}" << std::flush; }
    s << std::endl;
  }
  return s;
}

//=============================================================================
/*!  */
template<long n>
inline void zhematrix_small<n>::write(const char* filename) const
{VERBOSE_REPORT;
  std::ofstream ofs(filename, std::ios::trunc);
  ofs.setf(std::cout.flags());
  ofs.precision(std::cout.precision());
  ofs.width(std::cout.width());
  ofs.fill(std::cout.fill());
  ofs << "#zhematrix" << " " << n << std::endl;
  for(long i=0; i<n; i++){
    for(long j=0; j<=i; j++){
      ofs << operator()(i,j) << " ";
    }
    ofs << std::endl;
  }
  ofs.close();
}

//=============================================================================
/*!  */
template<long n>
inline void zhematrix_small<n>::read(const char* filename)
{VERBOSE_REPORT;
  std::ifstream s(filename);
  if(!s){
    ERROR_REPORT;
    std::cerr << "The file \"" << filename << "\" can not be opened." << std::endl;
    exit(1);
  }
  
  std::string id;
  s >> id;
  if( id != "zhematrix" && id != "#zhematrix" ){
    ERROR_REPORT;
    std::cerr << "The type name of the file \"" << filename << "\" is not zhematrix." << std::endl
              << "Its type name was " << id << " ." << std::endl;
    exit(1);
  }
  
  long _n;
  s >> _n;
  if(n!=_n){
    ERROR_REPORT;
    std::cerr << "Matrix size is invalid." << std::endl;
    exit(1);
  }
  
  for(long i=0; i<n; i++){
    for(long j=0; j<=i; j++ ){
      s >> operator()(i,j);
    }
  }
  if(s.eof()){
    ERROR_REPORT;
    std::cerr << "There is something is wrong with the file \"" << filename << "\"." << std::endl
              << "Most likely, there is a lack of data components, or a linefeed code or space code is missing at the end of the last line." << std::endl;
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
template<long n>
inline zhematrix_small<n>& zhematrix_small<n>::zero()
{VERBOSE_REPORT;
  for(long i=0; i<n; i++){
    for(long j=0; j<=i; j++){
      (*this)(i,j) =0.;
    }
  }
  return *this;
}

//=============================================================================
/*!  */
template<long n>
inline zhematrix_small<n>& zhematrix_small<n>::identity()
{VERBOSE_REPORT;
  zero();
  for(long k=0; k<n; k++){
    (*this)(k,k) =1.;
  }
  return *this;
}

//=============================================================================
/*!  */
template<long n>
inline void idamax(long& I, long& J, const zhematrix_small<n>& A)
{VERBOSE_REPORT;
  double max(-1.);
  for(int i=0; i<n; i++){
    for(int j=0; j<=i; j++){
      if( max<fabs(A(i,j)) ){
        I=i;
        J=j;
        max =fabs(A(i,j));
      }
    }
  }
  return;  
}

//=============================================================================
/*!  */
template<long n>
inline comple damax(const zhematrix_small<n>& A)
{VERBOSE_REPORT;
  long i(0),j(0);
  idamax(i,j,A);
  return A(i,j);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////


//=============================================================================
/*!  */
template<long n>
inline zhematrix_small<n>& operator+=(zhematrix_small<n>& A, const zhematrix_small<n>& B)
{VERBOSE_REPORT;
  for(long k=0; k<(n*(n+1))/2; k++){
    A.array[k] +=B.array[k];
  }
  return A;
}

//=============================================================================
/*!  */
template<long n>
inline zhematrix_small<n>& operator-=(zhematrix_small<n>& A, const zhematrix_small<n>& B)
{VERBOSE_REPORT;
  for(long k=0; k<(n*(n+1))/2; k++){
    A.array[k] -=B.array[k];
  }
  return A;
}

//=============================================================================
/*!  */
template<long n>
inline zhematrix_small<n>& operator*=(zhematrix_small<n>& A, const double& v)
{VERBOSE_REPORT;
  for(long i=0; i<n; i++){
    for(long j=0; j<=i; j++){
      A(i,j) *=v;
    }
  }
  return A;
}

//=============================================================================
/*!  */
template<long n>
inline zhematrix_small<n>& operator/=(zhematrix_small<n>& A, const double& v)
{VERBOSE_REPORT;
  for(long i=0; i<n; i++){
    for(long j=0; j<=i; j++){
      A(i,j) /=v;
    }
  }
  return A;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! unary */
template<long n>
inline const zhematrix_small<n>& operator+(const zhematrix_small<n>& A)
{VERBOSE_REPORT;
  return A;
}

//=============================================================================
/*! unary */
template<long n>
inline zhematrix_small<n> operator-(const zhematrix_small<n>& A)
{VERBOSE_REPORT;
  zhematrix_small<n> X;
  for(long i=0; i<n; i++){
    for(long j=0; j<=i; j++){
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
template<long n>
inline zgematrix_small<n,n> operator+(const zhematrix_small<n>& A, const zgematrix_small<n,n>& B)
{VERBOSE_REPORT;
  zgematrix_small<n,n> X;
  for(long i=0; i<n; i++){
    for(long j=0; j<i; j++){
      X(i,j) =A(i,j)+B(i,j);
    }
    for(long j=i; j<n; j++){
      X(i,j) =A(j,i)+B(i,j);
    }
  }
  return X;
}

//=============================================================================
/*!  */
template<long n>
inline zhematrix_small<n> operator+(const zhematrix_small<n>& A, const zhematrix_small<n>& B)
{VERBOSE_REPORT;
  zhematrix_small<n> X;
  for(long i=0; i<n; i++){
    for(long j=0; j<=i; j++){
      X(i,j) =A(i,j)+B(i,j);
    }
  }
  return X;
}


//=============================================================================
/*!  */
template<long n>
inline zgematrix_small<n,n> operator-(const zhematrix_small<n>& A, const zgematrix_small<n,n>& B)
{VERBOSE_REPORT;
  zgematrix_small<n,n> X;
  for(long i=0; i<n; i++){
    for(long j=0; j<i; j++){
      X(i,j) =A(i,j)-B(i,j);
    }
    for(long j=i; j<n; j++){
      X(i,j) =A(j,i)-B(i,j);
    }
  }
  return X;
}

//=============================================================================
/*!  */
template<long n>
inline zhematrix_small<n> operator-(const zhematrix_small<n>& A, const zhematrix_small<n>& B)
{VERBOSE_REPORT;
  zhematrix_small<n> X;
  for(long i=0; i<n; i++){
    for(long j=0; j<=i; j++){
      X(i,j) =A(i,j)-B(i,j);
    }
  }
  return X;
}

//=============================================================================
/*!  */
template<long n>
inline zcovector_small<n> operator*(const zhematrix_small<n>& A, const zcovector_small<n>& B)
{VERBOSE_REPORT;
  zcovector_small<n> C(0.);
  for(long i=0; i<n; i++){
    for(long j=0; j<i; j++){
      C(i) +=A(i,j)*B(j);
    }
    for(long j=i; j<n; j++){
      C(i) +=A(j,i)*B(j);
    }
  }
  return C;
}

//=============================================================================
/*!  */
template<long m, long n>
inline zgematrix_small<m,n> operator*(const zhematrix_small<m>& A, const zgematrix_small<m,n>& B)
{VERBOSE_REPORT;
  zgematrix_small<m,n> X(0.);
  for(long i=0; i<m; i++){
    for(long j=0; j<n; j++){
      for(long k=0; k<i; k++){
        X(i,j) +=A(i,k)*B(k,j);
      }
      for(long k=i; k<m; k++){
        X(i,j) +=A(k,i)*B(k,j);
      }
    }
  }
  return X;
}

//=============================================================================
/*!  */
template<long n>
inline zgematrix_small<n,n> operator*(const zhematrix_small<n>& A, const zhematrix_small<n>& B)
{VERBOSE_REPORT;
  zgematrix_small<n,n> X(0.);
  for(long i=0; i<n; i++){
    for(long j=0; j<i; j++){
      for(long k=0; k<j; k++){
        X(i,j) +=A(i,k)*B(j,k);
      }
      for(long k=j; k<i; k++){
        X(i,j) +=A(i,k)*B(k,j);
      }
      for(long k=i; k<n; k++){
        X(i,j) +=A(k,i)*B(k,j);
      }
    }
    for(long j=i; j<n; j++){
      for(long k=0; k<i; k++){
        X(i,j) +=A(i,k)*B(j,k);
      }
      for(long k=i; k<j; k++){
        X(i,j) +=A(k,i)*B(j,k);
      }
      for(long k=j; k<n; k++){
        X(i,j) +=A(k,i)*B(k,j);
      }
    }
  }
  return X;
}

//=============================================================================
/*!  */
template<long n>
inline zhematrix_small<n> operator*(const zhematrix_small<n>& A, const double& v)
{VERBOSE_REPORT;
  zhematrix_small<n> C;
  for(long i=0; i<n; i++){
    for(long j=0; j<=i; j++){
      C(i,j) =A(i,j)*v;
    }
  }
  return C;
}

//=============================================================================
/*!  */
template<long n>
inline zhematrix_small<n> operator/(const zhematrix_small<n>& A, const double& v)
{VERBOSE_REPORT;
  zhematrix_small<n> C;
  for(long i=0; i<n; i++){
    for(long j=0; j<=i; j++){
      C(i,j) =A(i,j)/v;
    }
  }
  return C;
}
