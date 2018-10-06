//=============================================================================
//! Samll Complex Double-precision General Dence Matrix Class
template<long m, long n> class zgematrix_small
{
public:
  ///////////////////////////////////////////////
  /////////////////// objects ///////////////////
  ///////////////////////////////////////////////
  comple array[m*n];
  
  ///////////////////////////////////////////////
  ///////////////// constructors ////////////////
  ///////////////////////////////////////////////
  inline zgematrix_small();
  inline explicit zgematrix_small(const zgematrix&);
  inline zgematrix_small(const comple&);
  inline ~zgematrix_small();
  
  ///////////////////////////////////////////////
  ////////////////// functions //////////////////
  ///////////////////////////////////////////////
  //////// cast ////////
  inline _zgematrix to_zgematrix() const;
  
  //////// io ////////
  inline comple& operator()(const long& i, const long& j);
  inline comple  operator()(const long& i, const long& j) const;
  inline zgematrix_small<m,n>& set(const long& i, const long& j, const comple& v);
  template<long _m, long _n> inline friend std::ostream& operator<<(std::ostream&, const zgematrix_small<_m,_n>&);
  inline void read(const char* filename);
  inline void write(const char* filename) const;
  
  //////// calc ////////
  template<long _m, long _n> inline friend zgematrix_small<n,m> t(const zgematrix_small<m,n>&);
  
  //////// misc ////////
  inline zgematrix_small<m,n>& zero();
  inline zgematrix_small<m,n>& identity();
  inline zcovector_small<m> col(const long& j) const;
  inline zrovector_small<n> row(const long& i) const;

  ///////////////////////////////////////////////
  ///////////// numerical operators /////////////
  ///////////////////////////////////////////////
  //////// = ////////
  template<long M, long N> inline zgematrix_small<M,N>& operator= (const  zgematrix_small<M,N>&);
  //////// += ////////
  template<long M, long N> inline friend zgematrix_small<M,N>& operator+=(zgematrix_small<M,N>&, const zgematrix_small<M,N>&);
  //////// -= ////////
  template<long M, long N> inline friend zgematrix_small<M,N>& operator-=(zgematrix_small<M,N>&, const zgematrix_small<M,N>&);
  //////// *= ////////
  template<long M, long L, long N> inline friend zgematrix_small<M,N>& operator*=(zgematrix_small<M,L>&, const zgematrix_small<L,N>&);
  template<long M, long N> inline friend zgematrix_small<M,N>& operator*=(zgematrix_small<M,N>&, const               comple&);
  //////// /= ////////
  template<long M, long N> inline friend zgematrix_small<M,N>& operator/=(zgematrix_small<M,N>&, const               comple&);
  //////// unary ////////
  template<long M, long N> inline friend const zgematrix_small<M,N>& operator+(const zgematrix_small<M,N>&);
  template<long M, long N> inline friend zgematrix_small<M,N> operator-(const zgematrix_small<M,N>&);
  //////// + ////////
  template<long M, long N> inline friend zgematrix_small<M,N> operator+(const zgematrix_small<M,N>&, const zgematrix_small<M,N>&);
  template<long M, long N> inline friend zgematrix_small<M,N> operator+(const zgematrix_small<M,N>&, const zhematrix_small< N >&);
  //////// - ////////
  template<long M, long N> inline friend zgematrix_small<M,N> operator-(const zgematrix_small<M,N>&, const zgematrix_small<M,N>&);
  template<long M, long N> inline friend zgematrix_small<M,N> operator-(const zgematrix_small<M,N>&, const zhematrix_small< N >&);
  //////// * ////////
  template<long M, long N> inline friend zcovector_small< M > operator*(const zgematrix_small<M,N>&, const zcovector_small< N >&);
  template<long M, long L, long N> inline friend zgematrix_small<M,N> operator*(const zgematrix_small<M,L>&, const zgematrix_small<L,N>&);
  template<long M, long N> inline friend zgematrix_small<M,N> operator*(const zgematrix_small<M,N>&, const zhematrix_small< N >&);
  template<long M, long N> inline friend zgematrix_small<M,N> operator*(const zgematrix_small<M,N>&, const               comple&);
  //////// / ////////
  template<long M, long N> inline friend zgematrix_small<M,N> operator/(const zgematrix_small<M,N>&, const               comple&);
  //////// comple ////////
  template<long M, long N> inline friend zgematrix_small<M,N> operator*(const               comple&, const zgematrix_small<M,N>&);
};

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

inline comple det(const zgemat2&);
inline zgemat2 inv(const zgemat2&);
inline comple det(const zgemat3&);
inline zgemat3 inv(const zgemat3&);
