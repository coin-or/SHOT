//=============================================================================
//! (DO NOT USE) Smart-temporary Complex Double-precision Row Vector Class
class _zrovector
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
  inline _zrovector();
  inline _zrovector(const _zrovector&);
  inline ~_zrovector(); //destructor
  
  ///////////////////////////////////////////////
  ////////////////// functions //////////////////
  ///////////////////////////////////////////////
  //////// cast ////////
  
  //////// io ////////
  inline comple& operator()(const long&) const;
  inline friend std::ostream& operator<<(std::ostream&, const _zrovector&);
  inline void write(const char*) const;
  
  //////// calc ////////
  inline friend _zcovector t(const _zrovector&);
  inline friend _zrovector conj(const _zrovector&);
  inline friend _zcovector conjt(const _zrovector&);
  inline friend double nrm2(const _zrovector&);
  inline friend long idamax(const _zrovector&);
  inline friend comple damax(const _zrovector&);

  //////// misc ////////
  inline void nullify() const;
  inline void destroy() const;
  
  ///////////////////////////////////////////////
  ///////////// numerical operators /////////////
  ///////////////////////////////////////////////
  //////// unary ////////
  inline friend const _zrovector& operator+(const _zrovector&);
  inline friend _zrovector operator-(const _zrovector&);
  
  //////// + ////////
  inline friend _zrovector operator+(const _zrovector&, const  zrovector&);
  inline friend _zrovector operator+(const _zrovector&, const _zrovector&);
  
  //////// - ////////
  inline friend _zrovector operator-(const _zrovector&, const  zrovector&);
  inline friend _zrovector operator-(const _zrovector&, const _zrovector&);
  
  //////// * ////////
  inline friend     comple operator*(const _zrovector&, const  zcovector&);
  inline friend     comple operator*(const _zrovector&, const _zcovector&);
  inline friend _zrovector operator*(const _zrovector&, const  zgematrix&);
  inline friend _zrovector operator*(const _zrovector&, const _zgematrix&);
  inline friend _zrovector operator*(const _zrovector&, const  zhematrix&);
  inline friend _zrovector operator*(const _zrovector&, const _zhematrix&);
  inline friend _zrovector operator*(const _zrovector&, const  zgbmatrix&);
  inline friend _zrovector operator*(const _zrovector&, const _zgbmatrix&);
  inline friend _zrovector operator*(const _zrovector&, const  zgsmatrix&);
  inline friend _zrovector operator*(const _zrovector&, const _zgsmatrix&);
  inline friend _zrovector operator*(const _zrovector&, const  zhsmatrix&);
  inline friend _zrovector operator*(const _zrovector&, const _zhsmatrix&);
  inline friend _zrovector operator*(const _zrovector&, const     double&);
  inline friend _zrovector operator*(const _zrovector&, const     comple&);
  
  //////// / ////////
  inline friend _zrovector operator/(const _zrovector&, const     double&);
  inline friend _zrovector operator/(const _zrovector&, const     comple&);
  
  //////// % ////////
  inline friend     comple operator%(const _zrovector&, const  zrovector&);
  inline friend     comple operator%(const _zrovector&, const _zrovector&);
  
  //////// double, complex ////////
  inline friend _zrovector operator*(const     double&, const _zrovector&);
  inline friend _zrovector operator*(const     comple&, const _zrovector&);
};
