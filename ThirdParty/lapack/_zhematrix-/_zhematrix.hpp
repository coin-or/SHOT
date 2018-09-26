//=============================================================================
//! (DO NOT USE) Smart-temporary Complex Double-precision Hermitian Matrix Class
class _zhematrix
{
public:
  ///////////////////////////////////////////////
  /////////////////// objects ///////////////////
  ///////////////////////////////////////////////
  long const& m;//!< matrix row and size
  //mutable long const& m;//!< matrix row and size
  mutable long n; //!< matrix column size
  mutable comple* array; //!< 1D array to store matrix data
  mutable comple** darray; //!< array of pointers of column head addresses
  
  ///////////////////////////////////////////////
  ///////////////// constructors ////////////////
  ///////////////////////////////////////////////
  inline _zhematrix();
  inline _zhematrix(const _zhematrix&);
  inline ~_zhematrix(); //destructor
  
  ///////////////////////////////////////////////
  ////////////////// functions //////////////////
  ///////////////////////////////////////////////
  //////// cast ////////
  inline _zgematrix to_zgematrix() const;
  
  //////// io ////////
  inline zhecomplex operator()(const long&, const long&) const;
  inline friend std::ostream& operator<<(std::ostream&, const zhematrix&);
  inline void write(const char*) const;
  
  //////// misc ////////
  inline void nullify() const;
  inline void destroy() const;
  inline void complete() const;
  
  //////// calc ////////
  inline friend _zhematrix t(const _zhematrix&);
  inline friend _zgematrix i(const _zhematrix&);
  
  ///////////////////////////////////////////////
  ///////////// numerical operators /////////////
  ///////////////////////////////////////////////
  //////// unary ////////
  inline friend const _zhematrix& operator+(const _zhematrix&);
  inline friend _zhematrix operator-(const _zhematrix&);
  
  //////// + ////////
  inline friend _zgematrix operator+(const _zhematrix&, const  zgematrix&);
  inline friend _zgematrix operator+(const _zhematrix&, const _zgematrix&);
  inline friend _zhematrix operator+(const _zhematrix&, const  zhematrix&);
  inline friend _zhematrix operator+(const _zhematrix&, const _zhematrix&);
  inline friend _zgematrix operator+(const _zhematrix&, const  zgbmatrix&);
  inline friend _zgematrix operator+(const _zhematrix&, const _zgbmatrix&);
  inline friend _zgematrix operator+(const _zhematrix&, const  zgsmatrix&);
  inline friend _zgematrix operator+(const _zhematrix&, const _zgsmatrix&);
  inline friend _zhematrix operator+(const _zhematrix&, const  zhsmatrix&);
  inline friend _zhematrix operator+(const _zhematrix&, const _zhsmatrix&);
  
  //////// - ////////
  inline friend _zgematrix operator-(const _zhematrix&, const  zgematrix&);  
  inline friend _zgematrix operator-(const _zhematrix&, const _zgematrix&);
  inline friend _zhematrix operator-(const _zhematrix&, const  zhematrix&);
  inline friend _zhematrix operator-(const _zhematrix&, const _zhematrix&);
  inline friend _zgematrix operator-(const _zhematrix&, const  zgbmatrix&);  
  inline friend _zgematrix operator-(const _zhematrix&, const _zgbmatrix&);
  inline friend _zgematrix operator-(const _zhematrix&, const  zgsmatrix&);
  inline friend _zgematrix operator-(const _zhematrix&, const _zgsmatrix&);
  inline friend _zhematrix operator-(const _zhematrix&, const  zhsmatrix&);
  inline friend _zhematrix operator-(const _zhematrix&, const _zhsmatrix&);
  
  //////// * ////////
  inline friend _zcovector operator*(const _zhematrix&, const  zcovector&);
  inline friend _zcovector operator*(const _zhematrix&, const _zcovector&);
  inline friend _zgematrix operator*(const _zhematrix&, const  zgematrix&);
  inline friend _zgematrix operator*(const _zhematrix&, const _zgematrix&);
  inline friend _zgematrix operator*(const _zhematrix&, const  zhematrix&);
  inline friend _zgematrix operator*(const _zhematrix&, const _zhematrix&);
  inline friend _zgematrix operator*(const _zhematrix&, const  zgbmatrix&);
  inline friend _zgematrix operator*(const _zhematrix&, const _zgbmatrix&);
  inline friend _zgematrix operator*(const _zhematrix&, const  zgsmatrix&);
  inline friend _zgematrix operator*(const _zhematrix&, const _zgsmatrix&);
  inline friend _zgematrix operator*(const _zhematrix&, const  zhsmatrix&);
  inline friend _zgematrix operator*(const _zhematrix&, const _zhsmatrix&);
  inline friend _zhematrix operator*(const _zhematrix&, const     double&);
  inline friend _zgematrix operator*(const _zhematrix&, const     comple&);
  
  //////// / ////////
  inline friend _zhematrix operator/(const _zhematrix&, const     double&);
  inline friend _zgematrix operator/(const _zhematrix&, const     comple&);
  
  //////// double, complex ////////
  inline friend _zhematrix operator*(const     double&, const _zhematrix&);
  inline friend _zgematrix operator*(const     comple&, const _zhematrix&);
};
