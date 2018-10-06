//=============================================================================
//! Samll Real Double-precision Row Vector Class
template<long l> class drovector_small
{
public:
  ///////////////////////////////////////////////
  /////////////////// objects ///////////////////
  ///////////////////////////////////////////////
  double array[l];
  
  ///////////////////////////////////////////////
  ///////////////// constructors ////////////////
  ///////////////////////////////////////////////
  inline drovector_small();
  inline explicit drovector_small(const drovector&);
  inline drovector_small(const double&);
  inline drovector_small(const double&, const double&);
  inline drovector_small(const double&, const double&, const double&);
  inline ~drovector_small();
  
  ///////////////////////////////////////////////
  ////////////////// functions //////////////////
  ///////////////////////////////////////////////
  //////// cast ////////
  inline _drovector to_drovector() const;
  
  //////// io ////////
  inline double& operator()(const long&);
  inline double  operator()(const long&) const;
  inline drovector_small<l>& set(const long&, const double&);
  template<long _l> inline friend std::ostream& operator<<(std::ostream&, const drovector_small<_l>&);
  inline void read(const char* filename);
  inline void write(const char* filename) const;
  
  //////// calc ////////
  template<long _l> inline friend dcovector_small<_l> t(const drovector_small<_l>&);
  template<long _l> inline friend double nrm2(const drovector_small<_l>&);
  template<long _l> inline friend long idamax(const drovector_small<_l>&);
  template<long _l> inline friend double damax(const drovector_small<_l>&);
  
  //////// misc ////////
  inline drovector_small<l>& zero();
  
  ///////////////////////////////////////////////
  ///////////// numerical operators /////////////
  ///////////////////////////////////////////////
  //////// = ////////
  template<long L> inline drovector_small<L>& operator= (const  drovector_small<L>&);
  //////// += ////////
  template<long L> inline friend drovector_small<L>& operator+=(drovector_small<L>&, const drovector_small<L>&);
  //////// -= ////////
  template<long L> inline friend drovector_small<L>& operator-=(drovector_small<L>&, const drovector_small<L>&);
  //////// *= ////////
  template<long L> inline friend drovector_small<L>& operator*=(drovector_small<L>&, const             double&);
  //////// /= ////////
  template<long L> inline friend drovector_small<L>& operator/=(drovector_small<L>&, const             double&);
  //////// unary ////////
  template<long L> inline friend const drovector_small<L>& operator+(const drovector_small<L>&);
  template<long L> inline friend drovector_small<L> operator-(const drovector_small<L>&);
  //////// + ////////
  template<long L> inline friend drovector_small<L> operator+(const drovector_small<L>&, const drovector_small<L>&);
  //////// - ////////
  template<long L> inline friend drovector_small<L> operator-(const drovector_small<L>&, const drovector_small<L>&);
  //////// * ////////
  template<long L> inline friend             double operator*(const drovector_small<L>&, const dcovector_small<L>&);
  template<long M, long N> inline friend drovector_small<N> operator*(const drovector_small<M>&, const dgematrix_small<M,N>&);
  template<long L> inline friend drovector_small<L> operator*(const drovector_small<L>&, const dsymatrix_small<L>&);
  template<long L> inline friend drovector_small<L> operator*(const drovector_small<L>&, const             double&);
  //////// / ////////
  template<long L> inline friend drovector_small<L> operator/(const drovector_small<L>&, const             double&);
  //////// double ////////
  template<long L> inline friend drovector_small<L> operator*(const             double&, const drovector_small<L>&);
};

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

inline double  operator/(const drovec2&, const drovec2&);
inline drovec3 operator/(const drovec3&, const drovec3&);
