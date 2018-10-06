//=============================================================================
//! Samll Real Double-precision Symmetric Matrix Class
template<long n> class dsymatrix_small
{
public:
  ///////////////////////////////////////////////
  /////////////////// objects ///////////////////
  ///////////////////////////////////////////////
  double array[(n*(n+1))/2];
  
  ///////////////////////////////////////////////
  ///////////////// constructors ////////////////
  ///////////////////////////////////////////////
  inline dsymatrix_small();
  inline explicit dsymatrix_small(const dsymatrix&);
  inline dsymatrix_small(const double&);
  inline ~dsymatrix_small();
  
  ///////////////////////////////////////////////
  ////////////////// functions //////////////////
  ///////////////////////////////////////////////
  //////// cast ////////
  inline dgematrix_small<n,n> to_dgematrix_small() const;
  inline dsymatrix to_dsymatrix() const;
  
  //////// io ////////
  inline double& operator()(const long& i, const long& j);
  inline double  operator()(const long& i, const long& j) const;
  inline dsymatrix_small<n>& set(const long&, const long&, const double&);
  template<long _n> inline friend std::ostream& operator<<(std::ostream&, const dsymatrix_small<_n>&);
  inline void read(const char* filename);
  inline void write(const char* filename) const;
  
  //////// misc ////////
  inline dsymatrix_small<n>& zero();
  inline dsymatrix_small<n>& identity();
  
  ///////////////////////////////////////////////
  ///////////// numerical operators /////////////
  ///////////////////////////////////////////////
  //////// = ////////
  template<long N> inline dsymatrix_small<N>& operator= (const dsymatrix_small<N>&);
  //////// += ////////
  template<long N> inline friend dsymatrix_small<N>& operator+=(dsymatrix_small<N>&, const dsymatrix_small<N>&);
  //////// -= ////////
  template<long N> inline friend dsymatrix_small<N>& operator-=(dsymatrix_small<N>&, const dsymatrix_small<N>&);
  //////// *= ////////
  template<long N> inline friend dsymatrix_small<N>& operator*=(dsymatrix_small<N>&, const dsymatrix_small<N>&);
  template<long N> inline friend dsymatrix_small<N>& operator*=(dsymatrix_small<N>&, const             double&);
  //////// /= ////////
  template<long N> inline friend dsymatrix_small<N>& operator/=(dsymatrix_small<N>&, const             double&);
  //////// unary ////////
  template<long N> inline friend const dsymatrix_small<N>& operator+(const dsymatrix_small<N>&);
  template<long N> inline friend dsymatrix_small< N > operator-(const dsymatrix_small<N>&);
  //////// + ////////
  template<long N> inline friend dgematrix_small<N,N> operator+(const dsymatrix_small<N>&, const dgematrix_small<N,N>&);
  template<long N> inline friend dsymatrix_small< N > operator+(const dsymatrix_small<N>&, const dsymatrix_small< N >&);
  //////// - ////////
  template<long N> inline friend dgematrix_small<N,N> operator-(const dsymatrix_small<N>&, const dgematrix_small<N,N>&);
  template<long N> inline friend dsymatrix_small< N > operator-(const dsymatrix_small<N>&, const dsymatrix_small< N >&);
  //////// * ////////
  template<long N> inline friend dcovector_small< N > operator*(const dsymatrix_small<N>&, const dcovector_small< N >&);
  template<long N> inline friend dgematrix_small<N,N> operator*(const dsymatrix_small<N>&, const dgematrix_small<N,N>&);
  template<long N> inline friend dgematrix_small<N,N> operator*(const dsymatrix_small<N>&, const dsymatrix_small< N >&);
  template<long N> inline friend dsymatrix_small< N > operator*(const dsymatrix_small<N>&, const               double&);
  //////// / ////////
  template<long N> inline friend dsymatrix_small< N > operator/(const dsymatrix_small<N>&, const               double&);
  //////// double ////////
  template<long N> inline friend dsymatrix_small< N > operator*(const             double&, const dsymatrix_small< N >&);
};

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

inline double det(const dsymat2&);
inline dsymat2 inv(const dsymat2&);
inline dsymat2 rotate(const dsymat2&, const double&);

inline double det(const dsymat3&);
inline dsymat3 inv(const dsymat3&);
inline dsymat3 rotate(const dsymat3&, const dquater&);
