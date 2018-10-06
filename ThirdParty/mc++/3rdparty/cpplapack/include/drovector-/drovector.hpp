//=============================================================================
//! Real Double-precision Row Vector Class
class drovector
{
public:
  ///////////////////////////////////////////////
  /////////////////// objects ///////////////////
  ///////////////////////////////////////////////
  long l; //!< vector size
  long cap; //!< vector capacity
  double* array; //!< 1D array to store vector data
  
  ///////////////////////////////////////////////
  ///////////////// constructors ////////////////
  ///////////////////////////////////////////////
  inline drovector();
  inline drovector(const drovector&);
  inline drovector(const _drovector&);
  inline drovector(const long&, const long=0);
  inline drovector(const char*);
  inline ~drovector(); //destructor
  
  ///////////////////////////////////////////////
  ////////////////// functions //////////////////
  ///////////////////////////////////////////////
  //////// cast ////////
  inline _zrovector to_zrovector() const;
  
  //////// io ////////
  inline double& operator()(const long&);
  inline double operator()(const long&) const;
  inline drovector& set(const long&, const double&); //const;
  inline friend std::ostream& operator<<(std::ostream&, const drovector&);
  inline void write(const char*) const;
  inline void read(const char*);
 
  //////// calc ////////
  inline friend _dcovector t(const drovector&);
  inline friend double nrm2(const drovector&);
  inline friend long idamax(const drovector&);
  inline friend double damax(const drovector&);

  //////// misc ////////
  inline void clear();
  inline drovector& zero();
  inline void chsign();
  inline void copy(const drovector&);
  inline void shallow_copy(const _drovector&);
  inline void alias(const drovector&);
  inline void unalias();
  inline drovector& resize(const long&, const long=0);
  inline void stretch(const long&);
  inline friend void swap(drovector&, drovector&);
  inline friend _drovector _(drovector&);
  
  ///////////////////////////////////////////////
  ///////////// numerical operators /////////////
  ///////////////////////////////////////////////
  //////// = ////////
  inline drovector& operator=(const  drovector&);
  inline drovector& operator=(const _drovector&);
  
  //////// += ///
  inline drovector& operator+=(const  drovector&);
  inline drovector& operator+=(const _drovector&);
  
  //////// -= ////////
  inline drovector& operator-=(const  drovector&);
  inline drovector& operator-=(const _drovector&);
  
  //////// *= ////////
  inline drovector& operator*=(const     double&);
  
  //////// /= ////////
  inline drovector& operator/=(const     double&);
  
  //////// unary ////////
  inline friend const drovector& operator+(const drovector&);
  inline friend _drovector operator-(const  drovector&);
  
  //////// + ////////
  inline friend _drovector operator+(const  drovector&, const  drovector&);
  inline friend _drovector operator+(const  drovector&, const _drovector&);
  
  //////// - ////////
  inline friend _drovector operator-(const  drovector&, const  drovector&);
  inline friend _drovector operator-(const  drovector&, const _drovector&);
  
  //////// * ////////
  inline friend     double operator*(const  drovector&, const  dcovector&);
  inline friend     double operator*(const  drovector&, const _dcovector&);
  inline friend _drovector operator*(const  drovector&, const  dgematrix&);
  inline friend _drovector operator*(const  drovector&, const _dgematrix&);
  inline friend _drovector operator*(const  drovector&, const  dsymatrix&);
  inline friend _drovector operator*(const  drovector&, const _dsymatrix&);
  inline friend _drovector operator*(const  drovector&, const  dgbmatrix&);
  inline friend _drovector operator*(const  drovector&, const _dgbmatrix&);
  inline friend _drovector operator*(const  drovector&, const  dgsmatrix&);
  inline friend _drovector operator*(const  drovector&, const _dgsmatrix&);
  inline friend _drovector operator*(const  drovector&, const  dssmatrix&);
  inline friend _drovector operator*(const  drovector&, const _dssmatrix&);
  inline friend _drovector operator*(const  drovector&, const     double&);
  
  //////// / ////////
  inline friend _drovector operator/(const  drovector&, const     double&);
  
  //////// % ////////
  inline friend     double operator%(const  drovector&, const  drovector&);
  inline friend     double operator%(const  drovector&, const _drovector&);
  
  //////// double ////////
  inline friend _drovector operator*(const     double&, const  drovector&);
};
