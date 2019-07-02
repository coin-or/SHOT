//=============================================================================
//! (DO NOT USE) Smart-temporary Complex Double-precision Column Vector Class
class _zcovector
{
public:
  ///////////////////////////////////////////////
  /////////////////// objects ///////////////////
  ///////////////////////////////////////////////
  mutable long l; //!< vector size
  mutable comple* array; //!< 1D array to store vector data
  
  ///////////////////////////////////////////////
  ///////////////// constructors ////////////////
  ///////////////////////////////////////////////
  inline _zcovector();
  inline _zcovector(const _zcovector&);
  inline ~_zcovector(); //destructor
  
  ///////////////////////////////////////////////
  ////////////////// functions //////////////////
  ///////////////////////////////////////////////
  //////// cast ////////
  
  //////// io ////////
  inline comple& operator()(const long&) const;
  inline friend std::ostream& operator<<(std::ostream&, const _zcovector&);
  inline void write(const char*) const;
  
  //////// calc ////////
  inline friend _zrovector t(const _zcovector&);
  inline friend _zcovector conj(const _zcovector&);
  inline friend _zrovector conjt(const _zcovector&);
  inline friend double nrm2(const _zcovector&);
  inline friend long idamax(const _zcovector&);
  inline friend comple damax(const _zcovector&);
  
  //////// misc ////////
  inline void nullify() const;
  inline void destroy() const;
  
  ///////////////////////////////////////////////
  ///////////// numerical operators /////////////
  ///////////////////////////////////////////////
  //////// unary ////////
  inline friend const _zcovector& operator+(const _zcovector&);
  inline friend _zcovector operator-(const _zcovector&);
  
  //////// + ////////
  inline friend _zcovector operator+(const _zcovector&, const  zcovector&);
  inline friend _zcovector operator+(const _zcovector&, const _zcovector&);
  
  //////// - ////////
  inline friend _zcovector operator-(const _zcovector&, const  zcovector&);
  inline friend _zcovector operator-(const _zcovector&, const _zcovector&);
  
  //////// * ////////
  inline friend _zgematrix operator*(const _zcovector&, const  zrovector&);
  inline friend _zgematrix operator*(const _zcovector&, const _zrovector&);
  inline friend _zcovector operator*(const _zcovector&, const     double&);
  inline friend _zcovector operator*(const _zcovector&, const     comple&);
  
  //////// / ////////
  inline friend _zcovector operator/(const _zcovector&, const     double&);
  inline friend _zcovector operator/(const _zcovector&, const     comple&);
  
  //////// % ////////
  inline friend     comple operator%(const _zcovector&, const  zcovector&);
  inline friend     comple operator%(const _zcovector&, const _zcovector&);
  
  //////// double, complex ////////
  inline friend _zcovector operator*(const     double&, const _zcovector&);
  inline friend _zcovector operator*(const     comple&, const _zcovector&);
};
