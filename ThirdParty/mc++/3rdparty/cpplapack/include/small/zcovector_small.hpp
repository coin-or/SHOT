//=============================================================================
//! Samll Complex Double-precision Column Vector Class
template<long l> class zcovector_small
{
public:
  ///////////////////////////////////////////////
  /////////////////// objects ///////////////////
  ///////////////////////////////////////////////
  comple array[l];
  
  ///////////////////////////////////////////////
  ///////////////// constructors ////////////////
  ///////////////////////////////////////////////
  inline zcovector_small();
  inline explicit zcovector_small(const zcovector&);
  inline zcovector_small(const comple&);
  inline zcovector_small(const comple&, const comple&);
  inline zcovector_small(const comple&, const comple&, const comple&);
  inline ~zcovector_small();
  
  ///////////////////////////////////////////////
  ////////////////// functions //////////////////
  ///////////////////////////////////////////////
  //////// cast ////////
  inline _zcovector to_zcovector() const;
  
  //////// io ////////
  inline comple& operator()(const long&);
  inline comple  operator()(const long&) const;
  inline zcovector_small<l>& set(const long&, const comple&);
  template<long _l> inline friend std::ostream& operator<<(std::ostream&, const zcovector_small<_l>&);
  inline void read(const char* filename);
  inline void write(const char* filename) const;
  
  //////// calc ////////
  template<long _l> inline friend zrovector_small<_l> t(const zcovector_small<_l>&);
  template<long _l> inline friend comple nrm2(const zcovector_small<_l>&);
  template<long _l> inline friend long idamax(const zcovector_small<_l>&);
  template<long _l> inline friend comple damax(const zcovector_small<_l>&);
  
  //////// misc ////////
  inline zcovector_small<l>& zero();
  
  ///////////////////////////////////////////////
  ///////////// numerical operators /////////////
  ///////////////////////////////////////////////
  //////// = ////////
  template<long L> inline zcovector_small<L>& operator= (const  zcovector_small<L>&);
  //////// += ////////
  template<long L> inline friend zcovector_small<L>& operator+=(zcovector_small<L>&, const zcovector_small<L>&);
  //////// -= ////////
  template<long L> inline friend zcovector_small<L>& operator-=(zcovector_small<L>&, const zcovector_small<L>&);
  //////// *= ////////
  template<long L> inline friend zcovector_small<L>& operator*=(zcovector_small<L>&, const             comple&);
  //////// /= ////////
  template<long L> inline friend zcovector_small<L>& operator/=(zcovector_small<L>&, const             comple&);
  //////// unary ////////
  template<long L> inline friend const zcovector_small<L>& operator+(const zcovector_small<L>&);
  template<long L> inline friend zcovector_small<L> operator-(const zcovector_small<L>&);
  //////// + ////////
  template<long L> inline friend zcovector_small<L> operator+(const zcovector_small<L>&, const zcovector_small<L>&);
  //////// - ////////
  template<long L> inline friend zcovector_small<L> operator-(const zcovector_small<L>&, const zcovector_small<L>&);
  //////// * ////////
  template<long M, long N> inline friend zgematrix_small<M,N> operator*(const zcovector_small<M>&, const zrovector_small<N>&);
  template<long L> inline friend zcovector_small<L> operator*(const zcovector_small<L>&, const             comple&);
  //////// / ////////
  template<long L> inline friend zcovector_small<L> operator/(const zcovector_small<L>&, const             comple&);
  //////// comple ////////
  template<long L> inline friend zcovector_small<L> operator*(const             comple&, const zcovector_small<L>&);
};

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
//////// zcovec2 ////////
inline comple  operator/(const zcovec2&, const zcovec2&);

//////// zcovec3 ////////
inline zcovec3 operator/(const zcovec3&, const zcovec3&);
inline zcovec3 operator/=(zcovec3&, const zcovec3&);
