//=============================================================================
//! Samll Complex Double-precision Row Vector Class
template<long l> class zrovector_small
{
public:
  ///////////////////////////////////////////////
  /////////////////// objects ///////////////////
  ///////////////////////////////////////////////
  comple array[l];
  
  ///////////////////////////////////////////////
  ///////////////// constructors ////////////////
  ///////////////////////////////////////////////
  inline zrovector_small();
  inline explicit zrovector_small(const zrovector&);
  inline zrovector_small(const comple&);
  inline zrovector_small(const comple&, const comple&);
  inline zrovector_small(const comple&, const comple&, const comple&);
  inline ~zrovector_small();
  
  ///////////////////////////////////////////////
  ////////////////// functions //////////////////
  ///////////////////////////////////////////////
  //////// cast ////////
  inline _zrovector to_zrovector() const;
  
  //////// io ////////
  inline comple& operator()(const long&);
  inline comple  operator()(const long&) const;
  inline zrovector_small<l>& set(const long&, const comple&);
  template<long _l> inline friend std::ostream& operator<<(std::ostream&, const zrovector_small<_l>&);
  inline void read(const char* filename);
  inline void write(const char* filename) const;
  
  //////// calc ////////
  template<long _l> inline friend zcovector_small<_l> t(const zrovector_small<_l>&);
  template<long _l> inline friend comple nrm2(const zrovector_small<_l>&);
  template<long _l> inline friend long idamax(const zrovector_small<_l>&);
  template<long _l> inline friend comple damax(const zrovector_small<_l>&);
  
  //////// misc ////////
  inline zrovector_small<l>& zero();
  
  ///////////////////////////////////////////////
  ///////////// numerical operators /////////////
  ///////////////////////////////////////////////
  //////// = ////////
  template<long L> inline zrovector_small<L>& operator= (const  zrovector_small<L>&);
  //////// += ////////
  template<long L> inline friend zrovector_small<L>& operator+=(zrovector_small<L>&, const zrovector_small<L>&);
  //////// -= ////////
  template<long L> inline friend zrovector_small<L>& operator-=(zrovector_small<L>&, const zrovector_small<L>&);
  //////// *= ////////
  template<long L> inline friend zrovector_small<L>& operator*=(zrovector_small<L>&, const             comple&);
  //////// /= ////////
  template<long L> inline friend zrovector_small<L>& operator/=(zrovector_small<L>&, const             comple&);
  //////// unary ////////
  template<long L> inline friend const zrovector_small<L>& operator+(const zrovector_small<L>&);
  template<long L> inline friend zrovector_small<L> operator-(const zrovector_small<L>&);
  //////// + ////////
  template<long L> inline friend zrovector_small<L> operator+(const zrovector_small<L>&, const zrovector_small<L>&);
  //////// - ////////
  template<long L> inline friend zrovector_small<L> operator-(const zrovector_small<L>&, const zrovector_small<L>&);
  //////// * ////////
  template<long L> inline friend             comple operator*(const zrovector_small<L>&, const zcovector_small<L>&);
  template<long M, long N> inline friend zrovector_small<N> operator*(const zrovector_small<M>&, const zgematrix_small<M,N>&);
  template<long L> inline friend zrovector_small<L> operator*(const zrovector_small<L>&, const dsymatrix_small<L>&);
  template<long L> inline friend zrovector_small<L> operator*(const zrovector_small<L>&, const             comple&);
  //////// / ////////
  template<long L> inline friend zrovector_small<L> operator/(const zrovector_small<L>&, const             comple&);
  //////// comple ////////
  template<long L> inline friend zrovector_small<L> operator*(const             comple&, const zrovector_small<L>&);
};

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

inline comple  operator/(const zrovec2&, const zrovec2&);
inline zrovec3 operator/(const zrovec3&, const zrovec3&);
