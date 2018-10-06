//=============================================================================
//! Complex Double-precision Column Vector Class
class zcovector
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
  inline zcovector();
  inline zcovector(const zcovector&);
  inline zcovector(const _zcovector&);
  inline zcovector(const long&);
  inline zcovector(const char*);
  inline ~zcovector(); //destructor
  
  ///////////////////////////////////////////////
  ////////////////// functions //////////////////
  ///////////////////////////////////////////////
  //////// cast ////////
  
  //////// io ////////
  inline comple& operator()(const long&);
  inline comple operator()(const long&) const;
  inline zcovector& set(const long&, const comple&);
  inline friend std::ostream& operator<<(std::ostream&, const zcovector&);
  inline void write(const char*) const;
  inline void read(const char*);
  
  //////// calc ////////
  inline friend _zrovector t(const zcovector&);
  inline friend _zcovector conj(const zcovector&);
  inline friend _zrovector conjt(const zcovector&);
  inline friend double nrm2(const zcovector&);
  inline friend long idamax(const zcovector&);
  inline friend comple damax(const zcovector&);
  
  //////// misc ////////
  inline void clear();
  inline zcovector& zero();
  inline void chsign();
  inline void copy(const zcovector&);
  inline void shallow_copy(const _zcovector&);
  inline void alias(const zcovector&);
  inline void unalias();
  inline void resize(const long&);
  inline friend void swap(zcovector&, zcovector&);
  inline friend _zcovector _(zcovector&);
  
  ///////////////////////////////////////////////
  ///////////// numerical operators /////////////
  ///////////////////////////////////////////////
  //////// = ////////
  inline zcovector& operator=(const  zcovector&);
  inline zcovector& operator=(const _zcovector&);
  
  //////// += ////////
  inline zcovector& operator+=(const  zcovector&);
  inline zcovector& operator+=(const _zcovector&);
  
  //////// -= ////////
  inline zcovector& operator-=(const  zcovector&);
  inline zcovector& operator-=(const _zcovector&);
  
  //////// *= ////////
  inline zcovector& operator*=(const     double&);
  inline zcovector& operator*=(const     comple&);
  
  //////// /= ////////
  inline zcovector& operator/=(const     double&);
  inline zcovector& operator/=(const     comple&);
  
  //////// unary ////////
  inline friend const zcovector& operator+(const zcovector&);
  inline friend _zcovector operator-(const  zcovector&);
  
  //////// + ////////
  inline friend _zcovector operator+(const  zcovector&, const  zcovector&);
  inline friend _zcovector operator+(const  zcovector&, const _zcovector&);
  
  //////// - ////////
  inline friend _zcovector operator-(const  zcovector&, const  zcovector&);
  inline friend _zcovector operator-(const  zcovector&, const _zcovector&);
  
  //////// * ////////
  inline friend _zgematrix operator*(const  zcovector&, const  zrovector&);
  inline friend _zgematrix operator*(const  zcovector&, const _zrovector&);
  inline friend _zcovector operator*(const  zcovector&, const     double&);
  inline friend _zcovector operator*(const  zcovector&, const     comple&);
  
  //////// / ////////
  inline friend _zcovector operator/(const  zcovector&, const     double&);
  inline friend _zcovector operator/(const  zcovector&, const     comple&);
  
  //////// % ////////
  inline friend     comple operator%(const  zcovector&, const  zcovector&);
  inline friend     comple operator%(const  zcovector&, const _zcovector&);
  
  //////// double, comple ////////
  inline friend _zcovector operator*(const     double&, const  zcovector&);
  inline friend _zcovector operator*(const     comple&, const  zcovector&);
};
