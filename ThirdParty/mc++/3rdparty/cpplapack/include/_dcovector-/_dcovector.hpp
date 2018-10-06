//=============================================================================
//! (DO NOT USE) Smart-temporary Real Double-precision Column Vector Class
class _dcovector
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
  inline _dcovector();
  inline _dcovector(const _dcovector&);
  inline ~_dcovector(); //destructor
  
  ///////////////////////////////////////////////
  ////////////////// functions //////////////////
  ///////////////////////////////////////////////
  //////// cast ////////
  inline _zcovector to_zcovector() const;
  
  //////// io ////////
  inline double& operator()(const long&) const;
  inline friend std::ostream& operator<<(std::ostream&, const _dcovector&);
  inline void write(const char*) const;
  
  //////// calc ////////
  inline friend _drovector t(const dcovector&);
  inline friend double nrm2(const dcovector&);
  inline friend long idamax(const dcovector&);
  inline friend double damax(const dcovector&);
  
  //////// misc ////////
  inline void nullify() const;
  inline void destroy() const;
  
  ///////////////////////////////////////////////
  ///////////// numerical operators /////////////
  ///////////////////////////////////////////////
  //////// unary ////////
  inline friend const _dcovector& operator+(const _dcovector&);
  inline friend _dcovector operator-(const _dcovector&);
  
  //////// + ////////
  inline friend _dcovector operator+(const _dcovector&, const  dcovector&);
  inline friend _dcovector operator+(const _dcovector&, const _dcovector&);
  
  //////// - ////////
  inline friend _dcovector operator-(const _dcovector&, const  dcovector&);
  inline friend _dcovector operator-(const _dcovector&, const _dcovector&);
  
  //////// * ////////
  inline friend _dgematrix operator*(const _dcovector&, const  drovector&);
  inline friend _dgematrix operator*(const _dcovector&, const _drovector&);
  inline friend _dcovector operator*(const _dcovector&, const     double&);
  
  //////// / ////////
  inline friend _dcovector operator/(const _dcovector&, const     double&);
  
  //////// % ////////
  inline friend     double operator%(const _dcovector&, const  dcovector&);
  inline friend     double operator%(const _dcovector&, const _dcovector&);
  
  //////// double ////////
  inline friend _dcovector operator*(const     double&, const _dcovector&);
};
