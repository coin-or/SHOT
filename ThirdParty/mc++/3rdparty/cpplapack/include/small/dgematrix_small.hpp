//=============================================================================
//! Samll Real Double-precision General Dence Matrix Class
template<long m, long n> class dgematrix_small
{
public:
  ///////////////////////////////////////////////
  /////////////////// objects ///////////////////
  ///////////////////////////////////////////////
  double array[m*n];
  
  ///////////////////////////////////////////////
  ///////////////// constructors ////////////////
  ///////////////////////////////////////////////
  inline dgematrix_small();
  inline explicit dgematrix_small(const dgematrix&);
  inline dgematrix_small(const double&);
  inline ~dgematrix_small();
  
  ///////////////////////////////////////////////
  ////////////////// functions //////////////////
  ///////////////////////////////////////////////
  //////// cast ////////
  inline _dgematrix to_dgematrix() const;
  
  //////// io ////////
  inline double& operator()(const long& i, const long& j);
  inline double  operator()(const long& i, const long& j) const;
  inline dgematrix_small<m,n>& set(const long& i, const long& j, const double& v);
  template<long _m, long _n> inline friend std::ostream& operator<<(std::ostream&, const dgematrix_small<_m,_n>&);
  inline void read(const char* filename);
  inline void write(const char* filename) const;
  
  //////// calc ////////
  template<long _m, long _n> inline friend dgematrix_small<n,m> t(const dgematrix_small<m,n>&);
  
  //////// misc ////////
  inline dgematrix_small<m,n>& zero();
  inline dgematrix_small<m,n>& identity();
  inline dcovector_small<m> col(const long& j) const;
  inline drovector_small<n> row(const long& i) const;

  ///////////////////////////////////////////////
  ///////////// numerical operators /////////////
  ///////////////////////////////////////////////
  //////// = ////////
  template<long M, long N> inline dgematrix_small<M,N>& operator= (const  dgematrix_small<M,N>&);
  //////// += ////////
  template<long M, long N> inline friend dgematrix_small<M,N>& operator+=(dgematrix_small<M,N>&, const dgematrix_small<M,N>&);
  //////// -= ////////
  template<long M, long N> inline friend dgematrix_small<M,N>& operator-=(dgematrix_small<M,N>&, const dgematrix_small<M,N>&);
  //////// *= ////////
  template<long M, long L, long N> inline friend dgematrix_small<M,N>& operator*=(dgematrix_small<M,L>&, const dgematrix_small<L,N>&);
  template<long M, long N> inline friend dgematrix_small<M,N>& operator*=(dgematrix_small<M,N>&, const               double&);
  //////// /= ////////
  template<long M, long N> inline friend dgematrix_small<M,N>& operator/=(dgematrix_small<M,N>&, const               double&);
  //////// unary ////////
  template<long M, long N> inline friend const dgematrix_small<M,N>& operator+(const dgematrix_small<M,N>&);
  template<long M, long N> inline friend dgematrix_small<M,N> operator-(const dgematrix_small<M,N>&);
  //////// + ////////
  template<long M, long N> inline friend dgematrix_small<M,N> operator+(const dgematrix_small<M,N>&, const dgematrix_small<M,N>&);
  template<long M, long N> inline friend dgematrix_small<M,N> operator+(const dgematrix_small<M,N>&, const dsymatrix_small< N >&);
  //////// - ////////
  template<long M, long N> inline friend dgematrix_small<M,N> operator-(const dgematrix_small<M,N>&, const dgematrix_small<M,N>&);
  template<long M, long N> inline friend dgematrix_small<M,N> operator-(const dgematrix_small<M,N>&, const dsymatrix_small< N >&);
  //////// * ////////
  template<long M, long N> inline friend dcovector_small< M > operator*(const dgematrix_small<M,N>&, const dcovector_small< N >&);
  template<long M, long L, long N> inline friend dgematrix_small<M,N> operator*(const dgematrix_small<M,L>&, const dgematrix_small<L,N>&);
  template<long M, long N> inline friend dgematrix_small<M,N> operator*(const dgematrix_small<M,N>&, const dsymatrix_small< N >&);
  template<long M, long N> inline friend dgematrix_small<M,N> operator*(const dgematrix_small<M,N>&, const               double&);
  //////// / ////////
  template<long M, long N> inline friend dgematrix_small<M,N> operator/(const dgematrix_small<M,N>&, const               double&);
  //////// double ////////
  template<long M, long N> inline friend dgematrix_small<M,N> operator*(const               double&, const dgematrix_small<M,N>&);
};

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

inline double det(const dgemat2&);
inline dgemat2 inv(const dgemat2&);
inline dgemat2 rotate(const dgemat2&, const double&);
inline dgemat2 t2m(const double&);

inline double det(const dgemat3&);
inline dgemat3 inv(const dgemat3&);
inline dgemat3 rotate(const dgemat3&, const dquater&);
inline dquater m2q(const dgemat3&);
