//=============================================================================
//! (DO NOT USE) Smart-temporary Complex Double-precision General Band Matrix Class
class _zgbmatrix
{
public:
  ///////////////////////////////////////////////
  /////////////////// objects ///////////////////
  ///////////////////////////////////////////////
  mutable long m; //!< matrix row size
  mutable long n; //!< matrix column size
  mutable long kl; //!< lower band width 
  mutable long ku; //!< upper band width 
  mutable comple* array; //!< 1D array to store matrix data
  mutable comple** darray; //!< array of pointers of column head addresses
  
  ///////////////////////////////////////////////
  ///////////////// constructors ////////////////
  ///////////////////////////////////////////////
  inline _zgbmatrix();
  inline _zgbmatrix(const _zgbmatrix&);
  inline ~_zgbmatrix();//destructor
  
  ///////////////////////////////////////////////
  ////////////////// functions //////////////////
  ///////////////////////////////////////////////
  //////// cast ////////
  inline _zgematrix to_zgematrix() const;
  
  //////// io ////////
  inline comple& operator()(const long&, const long&) const;
  inline friend std::ostream& operator<<(std::ostream&, const _zgbmatrix&);
  inline void write(const char*) const;
  
  //////// misc ////////
  inline void nullify() const;
  inline void destroy() const;
  
  //////// calc ////////
  inline friend _zgbmatrix t(const _zgbmatrix&);
  inline friend _zgematrix i(const _zgbmatrix&);
  inline friend _zgbmatrix conj(const _zgbmatrix&);
  inline friend _zgbmatrix conjt(const _zgbmatrix&);
  
  ///////////////////////////////////////////////
  ///////////// numerical operators /////////////
  ///////////////////////////////////////////////
  //////// unary ////////
  inline friend const _zgbmatrix& operator+(const _zgbmatrix&);
  inline friend _zgbmatrix operator-(const _zgbmatrix&);
  
  //////// + ////////
  inline friend _zgematrix operator+(const _zgbmatrix&, const  zgematrix&);
  inline friend _zgematrix operator+(const _zgbmatrix&, const _zgematrix&);
  inline friend _zgematrix operator+(const _zgbmatrix&, const  zhematrix&);
  inline friend _zgematrix operator+(const _zgbmatrix&, const _zhematrix&);
  inline friend _zgbmatrix operator+(const _zgbmatrix&, const  zgbmatrix&);
  inline friend _zgbmatrix operator+(const _zgbmatrix&, const _zgbmatrix&);
  inline friend _zgematrix operator+(const _zgbmatrix&, const  zgsmatrix&);
  inline friend _zgematrix operator+(const _zgbmatrix&, const _zgsmatrix&);
  inline friend _zgematrix operator+(const _zgbmatrix&, const  zhsmatrix&);
  inline friend _zgematrix operator+(const _zgbmatrix&, const _zhsmatrix&);
  
  //////// - ////////
  inline friend _zgematrix operator-(const _zgbmatrix&, const  zgematrix&);
  inline friend _zgematrix operator-(const _zgbmatrix&, const _zgematrix&);
  inline friend _zgematrix operator-(const _zgbmatrix&, const  zhematrix&);
  inline friend _zgematrix operator-(const _zgbmatrix&, const _zhematrix&);
  inline friend _zgbmatrix operator-(const _zgbmatrix&, const  zgbmatrix&);
  inline friend _zgbmatrix operator-(const _zgbmatrix&, const _zgbmatrix&);
  inline friend _zgematrix operator-(const _zgbmatrix&, const  zgsmatrix&);
  inline friend _zgematrix operator-(const _zgbmatrix&, const _zgsmatrix&);
  inline friend _zgematrix operator-(const _zgbmatrix&, const  zhsmatrix&);
  inline friend _zgematrix operator-(const _zgbmatrix&, const _zhsmatrix&);
  
  //////// * ////////
  inline friend _zcovector operator*(const _zgbmatrix&, const  zcovector&);
  inline friend _zcovector operator*(const _zgbmatrix&, const _zcovector&);
  inline friend _zgematrix operator*(const _zgbmatrix&, const  zgematrix&);
  inline friend _zgematrix operator*(const _zgbmatrix&, const _zgematrix&);
  inline friend _zgematrix operator*(const _zgbmatrix&, const  zhematrix&);
  inline friend _zgematrix operator*(const _zgbmatrix&, const _zhematrix&);
  inline friend _zgbmatrix operator*(const _zgbmatrix&, const  zgbmatrix&);
  inline friend _zgbmatrix operator*(const _zgbmatrix&, const _zgbmatrix&);
  inline friend _zgematrix operator*(const _zgbmatrix&, const  zgsmatrix&);
  inline friend _zgematrix operator*(const _zgbmatrix&, const _zgsmatrix&);
  inline friend _zgematrix operator*(const _zgbmatrix&, const  zhsmatrix&);
  inline friend _zgematrix operator*(const _zgbmatrix&, const _zhsmatrix&);
  inline friend _zgbmatrix operator*(const _zgbmatrix&, const     double&);
  inline friend _zgbmatrix operator*(const _zgbmatrix&, const     comple&);
  
  //////// / ////////
  inline friend _zgbmatrix operator/(const _zgbmatrix&, const     double&);
  inline friend _zgbmatrix operator/(const _zgbmatrix&, const     comple&);
  
  //////// double, complex ////////
  inline friend _zgbmatrix operator*(const     double&, const _zgbmatrix&);
  inline friend _zgbmatrix operator*(const     comple&, const _zgbmatrix&);
};
