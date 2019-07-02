//=============================================================================
/*!  */
template<long l>
inline _dcovector dcovector_small<l>::to_dcovector() const
{VERBOSE_REPORT;
  dcovector vec(l);
  for(long k=0; k<l; k++){
    vec(k)=(*this)(k);
  }
  return _(vec);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*!  */
template<long l>
inline double& dcovector_small<l>::operator()(const long& k)
{VERBOSE_REPORT;
  return array[k];
}

//=============================================================================
/*!  */
template<long l>
inline double dcovector_small<l>::operator()(const long& k) const
{VERBOSE_REPORT;
  return array[k];
}

//=============================================================================
/*!  */
template<long l>
inline dcovector_small<l>& dcovector_small<l>::set(const long& k, const double& v)
{VERBOSE_REPORT;
  (*this)(k) =v;
  return *this;
}

//=============================================================================
/*!  */
template<long l>
inline std::ostream& operator<<(std::ostream& s, const dcovector_small<l>& A)
{VERBOSE_REPORT;
  s << std::setiosflags(std::ios::showpos);
  for(long i=0; i<l; i++){
    s << A(i) << std::endl;
  }
  return s;
}

//=============================================================================
/*!  */
template<long l>
inline void dcovector_small<l>::write(const char* filename) const
{VERBOSE_REPORT;
  std::ofstream ofs(filename, std::ios::trunc);
  ofs.setf(std::cout.flags());
  ofs.precision(std::cout.precision());
  ofs.width(std::cout.width());
  ofs.fill(std::cout.fill());
  
  ofs << "#dcovector" << " " << l << std::endl;
  for(long k=0; k<l; k++){
    ofs << (*this)(k) << std::endl;
  }
  ofs.close();
}

//=============================================================================
/*!  */
template<long l>
inline void dcovector_small<l>::read(const char* filename)
{VERBOSE_REPORT;
  std::ifstream s( filename );
  if(!s){
    ERROR_REPORT;
    std::cerr << "The file \"" << filename << "\" can not be opened." << std::endl;
    exit(1);
  }
  
  std::string id;
  s >> id;
  if( id != "dcovector" && id != "#dcovector" ){
    ERROR_REPORT;
    std::cerr << "The type name of the file \"" << filename << "\" is not dcovector." << std::endl
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
template<long l>
inline drovector_small<l> t(const dcovector_small<l>& A)
{VERBOSE_REPORT;
  drovector_small<l> X;
  for(long i=0; i<l; i++){
    X(i) =A(i);
  }
  return X;
}

//=============================================================================
/*!  */
template<long l>
inline double nrm2(const dcovector_small<l>& A)
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
inline void idamax(long& K, const dcovector_small<l>& A)
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
inline double damax(const dcovector_small<l>& A)
{VERBOSE_REPORT;
  long k(0);
  idamax(k,A);
  return A(k);
}

//=============================================================================
/*!  */
template<long l>
inline dcovector_small<l> colon(const dcovector_small<l>& A, const dcovector_small<l>& B)
{VERBOSE_REPORT;
  dcovector_small<l> C;
  for(long i=0; i<l; i++){
    C(i) =A(i)*B(i);
  }
  return C;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*!  */
template<long l>
inline dcovector_small<l>& dcovector_small<l>::zero()
{VERBOSE_REPORT;
  for(long k=0; k<l; k++){
    array[k] =0.;
  }
  return *this;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*!  */
template<long l>
inline dcovector_small<l>& operator+=(dcovector_small<l>& A, const dcovector_small<l>& B)
{VERBOSE_REPORT;
  for(long i=0; i<l; i++){
    A(i) +=B(i);
  }
  return A;
}

//=============================================================================
/*!  */
template<long l>
inline dcovector_small<l>& operator-=(dcovector_small<l>& A, const dcovector_small<l>& B)
{VERBOSE_REPORT;
  for(long i=0; i<l; i++){
    A(i) -=B(i);
  }
  return A;
}

//=============================================================================
/*!  */
template<long l>
inline dcovector_small<l>& operator*=(dcovector_small<l>& A, const double& d)
{VERBOSE_REPORT;
  for(long i=0; i<l; i++){
    A(i) *=d;
  }
  return A;
}

//=============================================================================
/*!  */
template<long l>
inline dcovector_small<l>& operator/=(dcovector_small<l>& A, const double& d)
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
inline const dcovector_small<l>& operator+(const dcovector_small<l>& A)
{VERBOSE_REPORT;
  return A;
}

//=============================================================================
/*! unary */
template<long l>
inline dcovector_small<l> operator-(const dcovector_small<l>& A)
{VERBOSE_REPORT;
  dcovector_small<l> X;
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
inline dcovector_small<l> operator+(const dcovector_small<l>& A, const dcovector_small<l>& B)
{VERBOSE_REPORT;
  dcovector_small<l> X;
  for(long i=0; i<l; i++){
    X(i) =A(i)+B(i);
  }
  return X;
}

//=============================================================================
/*!  */
template<long l>
inline dcovector_small<l> operator-(const dcovector_small<l>& A, const dcovector_small<l>& B)
{VERBOSE_REPORT;
  dcovector_small<l> X;
  for(long i=0; i<l; i++){
    X(i) =A(i)-B(i);
  }
  return X;
}

//=============================================================================
/*!  */
template<long n>
inline dcovector_small<n> operator*(const dcovector_small<n>& A, const double& v)
{VERBOSE_REPORT;
  dcovector_small<n> C;
  for(long i=0; i<n; i++){
    C(i) =A(i)*v;
  }
  return C;
}

//=============================================================================
/*!  */
template<long m, long n>
inline dgematrix_small<m,n> operator*(const dcovector_small<m>& A, const drovector_small<n>& B)
{VERBOSE_REPORT;
  dgematrix_small<m,n> mat;
  for(long i=0; i<m; i++){
    for(long j=0; j<n; j++){
      mat(i,j) =A(i)*B(j);
    }
  }
  return mat;
}

//=============================================================================
/*!  */
template<long n>
inline dcovector_small<n> operator/(const dcovector_small<n>& A, const double& v)
{VERBOSE_REPORT;
  dcovector_small<n> C;
  for(long i=0; i<n; i++){
    C(i) =A(i)/v;
  }
  return C;
}

//=============================================================================
/*!  */
template<long l>
inline double operator%(const dcovector_small<l>& A, const dcovector_small<l>& B)
{VERBOSE_REPORT;
  double v(0.);
  for(long i=0; i<l; i++){
    v +=A(i)*B(i);
  }
  return v;
}
