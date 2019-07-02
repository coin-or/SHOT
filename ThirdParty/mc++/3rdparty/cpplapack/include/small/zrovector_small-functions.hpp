//=============================================================================
/*!  */
template<long l>
inline _zrovector zrovector_small<l>::to_zrovector() const
{VERBOSE_REPORT;
  zrovector vec(l);
  for(long k=0; k<l; k++){
    vec(k) =(*this)(k);
  }
  return _(vec);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*!  */
template<long l>
inline comple& zrovector_small<l>::operator()(const long& k)
{VERBOSE_REPORT;
  return array[k];
}

//=============================================================================
/*!  */
template<long l>
inline comple zrovector_small<l>::operator()(const long& k) const
{VERBOSE_REPORT;
  return array[k];
}

//=============================================================================
/*!  */
template<long l>
inline zrovector_small<l>& zrovector_small<l>::set(const long& k, const comple& v)
{VERBOSE_REPORT;
  (*this)(k) =v;
  return *this;
}

//=============================================================================
/*!  */
template<long l>
inline std::ostream& operator<<(std::ostream& s, const zrovector_small<l>& A)
{VERBOSE_REPORT;
  s << std::setiosflags(std::ios::showpos);
  for(long i=0; i<l; i++){
    s << " " << A(i) << std::flush;
  }
  s << std::endl;
  return s;
}

//=============================================================================
/*!  */
template<long l>
inline void zrovector_small<l>::write(const char* filename) const
{VERBOSE_REPORT;
  std::ofstream ofs(filename, std::ios::trunc);
  ofs.setf(std::cout.flags());
  ofs.precision(std::cout.precision());
  ofs.width(std::cout.width());
  ofs.fill(std::cout.fill());
  
  ofs << "#zrovector" << " " << l << std::endl;
  for(long k=0; k<l; k++){
    ofs << (*this)(k) << std::endl;
  }
  ofs.close();
}

//=============================================================================
/*!  */
template<long l>
inline void zrovector_small<l>::read(const char* filename)
{VERBOSE_REPORT;
  std::ifstream s( filename );
  if(!s){
    ERROR_REPORT;
    std::cerr << "The file \"" << filename << "\" can not be opened." << std::endl;
    exit(1);
  }
  
  std::string id;
  s >> id;
  if( id != "zrovector" && id != "#zrovector" ){
    ERROR_REPORT;
    std::cerr << "The type name of the file \"" << filename << "\" is not zrovector." << std::endl
              << "Its type name was " << id << " ." << std::endl;
    exit(1);
  }
  
  long _l;
  s >> _l;
  if(l!=_l){
    ERROR_REPORT;
    std::cerr << "Matrix size is invalid." << std::endl;
    exit(1);
  }
  for(long k=0; k<l; k++){
    s >> (*this)(k);
  }
  if(s.eof()){
    ERROR_REPORT;
    std::cerr << "There is something is wrong with the file \"" << filename << "\"." << std::endl
              << "Most likely, there is not enough data components, or a linefeed code or space code is missing at the end of the last line." << std::endl;
    exit(1);
  }
  
  s >> id;//tmp
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
template<long n>
inline zcovector_small<n> t(const zrovector_small<n>& A)
{VERBOSE_REPORT;
  zcovector_small<n> X;
  for(long i=0; i<n; i++){
    X(i)=A(i);
  }
  return X;
}

//=============================================================================
/*!  */
template<long l>
inline double nrm2(const zrovector_small<l>& A)
{VERBOSE_REPORT;
  double v(0);
  for(long i=0; i<l; i++){
    v+=A(i)*A(i);
  }
  return sqrt(v);
}

//=============================================================================
/*!  */
template<long l>
inline void idamax(long& K, const zrovector_small<l>& A)
{VERBOSE_REPORT;
  double max(-1.);
  for(int k=0; k<l; k++){
    if( max<fabs(A(k)) ){
      K=k;
      max =fabs(A(k));
    }
  }
  return;
}

//=============================================================================
/*!  */
template<long l>
inline comple damax(const zrovector_small<l>& A)
{VERBOSE_REPORT;
  long k(0);
  idamax(k,A);
  return A(k);
}

//=============================================================================
/*!  */
template<long l>
inline zrovector_small<l> colon(const zrovector_small<l>& A, const zrovector_small<l>& B)
{VERBOSE_REPORT;
  zrovector_small<l> C;
  for(long i=0; i<l; i++){
    C(i) =A(i)*B(i);
  }
  return C;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*!  */
template<long l>
inline zrovector_small<l>& zrovector_small<l>::zero()
{VERBOSE_REPORT;
  for(long k=0; k<l; k++){
    array[k] =0.;
  }
  return *this;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*!  */
template<long l>
inline zrovector_small<l>& operator+=(zrovector_small<l>& A, const zrovector_small<l>& B)
{VERBOSE_REPORT;
  for(long i=0; i<l; i++){
    A(i) +=B(i);
  }
  return A;
}

//=============================================================================
/*!  */
template<long l>
inline zrovector_small<l>& operator-=(zrovector_small<l>& A, const zrovector_small<l>& B)
{VERBOSE_REPORT;
  for(long i=0; i<l; i++){
    A(i) -=B(i);
  }
  return A;
}

//=============================================================================
/*!  */
template<long l>
inline zrovector_small<l>& operator*=(zrovector_small<l>& A, const comple& d)
{VERBOSE_REPORT;
  for(long i=0; i<l; i++){
    A(i) *=d;
  }
  return A;
}

//=============================================================================
/*!  */
template<long l>
inline zrovector_small<l>& operator/=(zrovector_small<l>& A, const comple& d)
{VERBOSE_REPORT;
  for(long i=0; i<l; i++){
    A(i) /=d;
  }
  return A;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! unary */
template<long l>
inline const zrovector_small<l>& operator+(const zrovector_small<l>& A)
{VERBOSE_REPORT;
  return A;
}

//=============================================================================
/*! unary */
template<long l>
inline zrovector_small<l> operator-(const zrovector_small<l>& A)
{VERBOSE_REPORT;
  zrovector_small<l> X;
  for(long i=0; i<l; i++){
    X(i) =-A(i);
  }
  return X;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*!  */
template<long l>
inline zrovector_small<l> operator+(const zrovector_small<l>& A, const zrovector_small<l>& B)
{VERBOSE_REPORT;
  zrovector_small<l> X;
  for(long i=0; i<l; i++){
    X(i) =A(i)+B(i);
  }
  return X;
}

//=============================================================================
/*!  */
template<long l>
inline zrovector_small<l> operator-(const zrovector_small<l>& A, const zrovector_small<l>& B)
{VERBOSE_REPORT;
  zrovector_small<l> X;
  for(long i=0; i<l; i++){
    X(i) =A(i)-B(i);
  }
  return X;
}

//=============================================================================
/*!  */
template<long l>
inline comple operator*(const zrovector_small<l>& A, const zcovector_small<l>& B)
{VERBOSE_REPORT;
  comple x =0.;
  for(long i=0; i<l; i++){
    x +=A(i)*B(i);
  }
  return x;
}

//=============================================================================
/*!  */
template<long m, long n>
inline zrovector_small<n> operator*(const zrovector_small<m>& A, const zgematrix_small<m,n>& B)
{VERBOSE_REPORT;
  zrovector_small<n> C(0.);
  for(long j=0; j<n; j++){
    for(long i=0; i<m; i++){
      C(j) +=A(i)*B(i,j);
    }
  }
  return C;
}

//=============================================================================
/*!  */
template<long l>
inline zrovector_small<l> operator*(const zrovector_small<l>& A, const zhematrix_small<l>& B)
{VERBOSE_REPORT;
  zrovector_small<l> C(0.);
  for(long j=0; j<l; j++){
    for(long i=0; i<j; i++){
      C(j) +=A(i)*B(j,i);
    }
    for(long i=j; i<l; i++){
      C(j) +=A(i)*B(i,j);
    }    
  }
  return C;
}

//=============================================================================
/*!  */
template<long l>
inline zrovector_small<l> operator*(const zrovector_small<l>& A, const comple& v)
{VERBOSE_REPORT;
  zrovector_small<l> C;
  for(long i=0; i<l; i++){
    C(i) =A(i)*v;
  }
  return C;
}

//=============================================================================
/*!  */
template<long l>
inline zrovector_small<l> operator/(const zrovector_small<l>& A, const comple& v)
{VERBOSE_REPORT;
  zrovector_small<l> C;
  for(long i=0; i<l; i++){
    C(i) =A(i)/v;
  }
  return C;
}
