//=============================================================================
//! Complex Double-precision Row Vector Class
class zrovector
{
public:
  ///////////////////////////////////////////////
  /////////////////// objects ///////////////////
  ///////////////////////////////////////////////
  long l; //!< vector size
  comple* array; //!< 1D array to store vector data
  
  ///////////////////////////////////////////////
  ///////////////// constructors ////////////////
  ///////////////////////////////////////////////
  inline zrovector();
  inline zrovector(const zrovector&);
  inline zrovector(const _zrovector&);
  inline zrovector(const long&);
  inline zrovector(const char*);
  inline ~zrovector(); //destructor
  
  ///////////////////////////////////////////////
  ////////////////// functions //////////////////
  ///////////////////////////////////////////////
  //////// cast ////////
  
  //////// io ////////
  inline comple& operator()(const long&);
  inline comple operator()(const long&) const;
  inline zrovector& set(const long&, const comple&);
  inline friend std::ostream& operator<<(std::ostream&, const zrovector&);
  inline void write(const char*) const;
  inline void read(const char*);
 
  //////// calc ////////
  inline friend _zcovector t(const zrovector&);
  inline friend _zrovector conj(const zrovector&);
  inline friend _zcovector conjt(const zrovector&);
  inline friend double nrm2(const zrovector&);
  inline friend long idamax(const zrovector&);
  inline friend comple damax(const zrovector&);

  //////// misc ////////
  inline void clear();
  inline zrovector& zero();
  inline void chsign();
  inline void copy(const zrovector&);
  inline void shallow_copy(const _zrovector&);
  inline void alias(const zrovector&);
  inline void unalias();
  inline void resize(const long&);
  inline friend void swap(zrovector&, zrovector&);
  inline friend _zrovector _(zrovector&);
  
  ///////////////////////////////////////////////
  ///////////// numerical operators /////////////
  ///////////////////////////////////////////////
  //////// = ////////
  inline zrovector& operator=(const  zrovector&);
  inline zrovector& operator=(const _zrovector&);
  
  //////// += ////////
  inline zrovector& operator+=(const  zrovector&);
  inline zrovector& operator+=(const _zrovector&);
  
  //////// -= ////////
  inline zrovector& operator-=(const  zrovector&);
  inline zrovector& operator-=(const _zrovector&);
  
  //////// *= ////////
  inline zrovector& operator*=(const     double&);
  inline zrovector& operator*=(const     comple&);
  
  //////// /= ////////
  inline zrovector& operator/=(const     double&);
  inline zrovector& operator/=(const     comple&);
  
  //////// unary ////////
  inline friend const zrovector& operator+(const zrovector&);
  inline friend _zrovector operator-(const  zrovector&);
  
  //////// + ////////
  inline friend _zrovector operator+(const  zrovector&, const  zrovector&);
  inline friend _zrovector operator+(const  zrovector&, const _zrovector&);
  
  //////// - ////////
  inline friend _zrovector operator-(const  zrovector&, const  zrovector&);
  inline friend _zrovector operator-(const  zrovector&, const _zrovector&);
  
  //////// * ////////
  inline friend     comple operator*(const  zrovector&, const  zcovector&);
  inline friend     comple operator*(const  zrovector&, const _zcovector&);
  inline friend _zrovector operator*(const  zrovector&, const  zgematrix&);
  inline friend _zrovector operator*(const  zrovector&, const _zgematrix&);
  inline friend _zrovector operator*(const  zrovector&, const  zhematrix&);
  inline friend _zrovector operator*(const  zrovector&, const _zhematrix&);
  inline friend _zrovector operator*(const  zrovector&, const  zgbmatrix&);
  inline friend _zrovector operator*(const  zrovector&, const _zgbmatrix&);
  inline friend _zrovector operator*(const  zrovector&, const  zgsmatrix&);
  inline friend _zrovector operator*(const  zrovector&, const _zgsmatrix&);
  inline friend _zrovector operator*(const  zrovector&, const  zhsmatrix&);
  inline friend _zrovector operator*(const  zrovector&, const _zhsmatrix&);
  inline friend _zrovector operator*(const  zrovector&, const     double&);
  inline friend _zrovector operator*(const  zrovector&, const     comple&);
  
  //////// / ////////
  inline friend _zrovector operator/(const  zrovector&, const     double&);
  inline friend _zrovector operator/(const  zrovector&, const     comple&);
  
  //////// % ////////
  inline friend     comple operator%(const  zrovector&, const  zrovector&);
  inline friend     comple operator%(const  zrovector&, const _zrovector&);
  
  //////// double, comple ////////
  inline friend _zrovector operator*(const     double&, const  zrovector&);
  inline friend _zrovector operator*(const     comple&, const  zrovector&);
};
