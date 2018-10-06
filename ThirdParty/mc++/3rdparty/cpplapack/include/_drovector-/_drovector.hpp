//=============================================================================
//! (DO NOT USE) Smart-temporary Real Double-precision Row Vector Class
class _drovector
{
public:
  ///////////////////////////////////////////////
  /////////////////// objects ///////////////////
  ///////////////////////////////////////////////
  mutable long l; //!< vector size
  mutable long cap; //!< vector capacity
  mutable double* array; //!< 1D array to store vector data
  
  ///////////////////////////////////////////////
  ///////////////// constructors ////////////////
  ///////////////////////////////////////////////
  inline _drovector();
  inline _drovector(const _drovector&);
  inline ~_drovector(); //destructor
  
  ///////////////////////////////////////////////
  ////////////////// functions //////////////////
  ///////////////////////////////////////////////
  //////// cast ////////
  inline _zrovector to_zrovector() const;
  
  //////// io ////////
  inline double& operator()(const long&) const;
  inline friend std::ostream& operator<<(std::ostream&, const _drovector&);
  inline void write(const char*) const;
  
  //////// calc ////////
  inline friend _dcovector t(const drovector&);
  inline friend double nrm2(const drovector&);
  inline friend long idamax(const drovector&);
  inline friend double damax(const drovector&);

  //////// misc ////////
  inline void nullify() const;
  inline void destroy() const;
  
  ///////////////////////////////////////////////
  ///////////// numerical operators /////////////
  ///////////////////////////////////////////////
  //////// unary ////////
  inline friend const _drovector& operator+(const _drovector&);
  inline friend _drovector operator-(const _drovector&);
  
  //////// + ////////
  inline friend _drovector operator+(const _drovector&, const  drovector&);
  inline friend _drovector operator+(const _drovector&, const _drovector&);
  
  //////// - ////////
  inline friend _drovector operator-(const _drovector&, const  drovector&);
  inline friend _drovector operator-(const _drovector&, const _drovector&);
  
  //////// * ////////
  inline friend     double operator*(const _drovector&, const  dcovector&);
  inline friend     double operator*(const _drovector&, const _dcovector&);
  inline friend _drovector operator*(const _drovector&, const  dgematrix&);
  inline friend _drovector operator*(const _drovector&, const _dgematrix&);
  inline friend _drovector operator*(const _drovector&, const  dsymatrix&);
  inline friend _drovector operator*(const _drovector&, const _dsymatrix&);
  inline friend _drovector operator*(const _drovector&, const  dgbmatrix&);
  inline friend _drovector operator*(const _drovector&, const _dgbmatrix&);
  inline friend _drovector operator*(const _drovector&, const  dgsmatrix&);
  inline friend _drovector operator*(const _drovector&, const _dgsmatrix&);
  inline friend _drovector operator*(const _drovector&, const  dssmatrix&);
  inline friend _drovector operator*(const _drovector&, const _dssmatrix&);
  inline friend _drovector operator*(const _drovector&, const     double&);
  
  //////// / ////////
  inline friend _drovector operator/(const _drovector&, const     double&);
  
  //////// % ////////
  inline friend     double operator%(const _drovector&, const  drovector&);
  inline friend     double operator%(const _drovector&, const _drovector&);
  
  //////// double ////////
  inline friend _drovector operator*(const     double&, const _drovector&);
};
