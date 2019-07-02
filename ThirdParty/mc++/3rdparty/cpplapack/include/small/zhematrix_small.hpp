//=============================================================================
//! Samll Complex Double-precision Symmetric Matrix Class
template<long n> class zhematrix_small
{
public:
  ///////////////////////////////////////////////
  /////////////////// objects ///////////////////
  ///////////////////////////////////////////////
  comple array[(n*(n+1))/2];
  
  ///////////////////////////////////////////////
  ///////////////// constructors ////////////////
  ///////////////////////////////////////////////
  inline zhematrix_small();
  inline explicit zhematrix_small(const zhematrix&);
  inline zhematrix_small(const comple&);
  inline ~zhematrix_small();
  
  ///////////////////////////////////////////////
  ////////////////// functions //////////////////
  ///////////////////////////////////////////////
  //////// cast ////////
  inline zgematrix_small<n,n> to_zgematrix_small() const;
  inline zhematrix to_zhematrix() const;
  
  //////// io ////////
  inline comple& operator()(const long& i, const long& j);
  inline comple  operator()(const long& i, const long& j) const;
  inline zhematrix_small<n>& set(const long&, const long&, const comple&);
  template<long _n> inline friend std::ostream& operator<<(std::ostream&, const zhematrix_small<_n>&);
  inline void read(const char* filename);
  inline void write(const char* filename) const;
  
  //////// misc ////////
  inline zhematrix_small<n>& zero();
  inline zhematrix_small<n>& identity();
  
  ///////////////////////////////////////////////
  ///////////// numerical operators /////////////
  ///////////////////////////////////////////////
  //////// = ////////
  template<long N> inline zhematrix_small<N>& operator= (const zhematrix_small<N>&);
  //////// += ////////
  template<long N> inline friend zhematrix_small<N>& operator+=(zhematrix_small<N>&, const zhematrix_small<N>&);
  //////// -= ////////
  template<long N> inline friend zhematrix_small<N>& operator-=(zhematrix_small<N>&, const zhematrix_small<N>&);
  //////// *= ////////
  template<long N> inline friend zhematrix_small<N>& operator*=(zhematrix_small<N>&, const zhematrix_small<N>&);
  template<long N> inline friend zhematrix_small<N>& operator*=(zhematrix_small<N>&, const             comple&);
  //////// /= ////////
  template<long N> inline friend zhematrix_small<N>& operator/=(zhematrix_small<N>&, const             comple&);
  //////// unary ////////
  template<long N> inline friend const zhematrix_small<N>& operator+(const zhematrix_small<N>&);
  template<long N> inline friend zhematrix_small< N > operator-(const zhematrix_small<N>&);
  //////// + ////////
  template<long N> inline friend zgematrix_small<N,N> operator+(const zhematrix_small<N>&, const zgematrix_small<N,N>&);
  template<long N> inline friend zhematrix_small< N > operator+(const zhematrix_small<N>&, const zhematrix_small< N >&);
  //////// - ////////
  template<long N> inline friend zgematrix_small<N,N> operator-(const zhematrix_small<N>&, const zgematrix_small<N,N>&);
  template<long N> inline friend zhematrix_small< N > operator-(const zhematrix_small<N>&, const zhematrix_small< N >&);
  //////// * ////////
  template<long N> inline friend zcovector_small< N > operator*(const zhematrix_small<N>&, const zcovector_small< N >&);
  template<long N> inline friend zgematrix_small<N,N> operator*(const zhematrix_small<N>&, const zgematrix_small<N,N>&);
  template<long N> inline friend zgematrix_small<N,N> operator*(const zhematrix_small<N>&, const zhematrix_small< N >&);
  template<long N> inline friend zhematrix_small< N > operator*(const zhematrix_small<N>&, const               comple&);
  //////// / ////////
  template<long N> inline friend zhematrix_small< N > operator/(const zhematrix_small<N>&, const               comple&);
  //////// comple ////////
  template<long N> inline friend zhematrix_small< N > operator*(const             comple&, const zhematrix_small< N >&);
};

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

inline comple det(const zhemat2&);
inline zhemat2 inv(const zhemat2&);
inline comple det(const zhemat3&);
inline zhemat3 inv(const zhemat3&);
