//=============================================================================
//! (DO NOT USE) Smart-temporary Complex Double-precision General Dence Matrix Class
class _zgematrix
{
public:
  ///////////////////////////////////////////////
  /////////////////// objects ///////////////////
  ///////////////////////////////////////////////
  mutable long m; //!< matrix row size
  mutable long n; //!< matrix column size
  mutable comple* array; //!< 1D array to store matrix data
  mutable comple** darray; //!< array of pointers of column head addresses

  ///////////////////////////////////////////////
  ///////////////// constructors ////////////////
  ///////////////////////////////////////////////
  inline _zgematrix();
  inline _zgematrix(const _zgematrix&);
  inline ~_zgematrix(); //destructor
  
  ///////////////////////////////////////////////
  ////////////////// functions //////////////////
  ///////////////////////////////////////////////
  //////// cast ////////
  
  //////// io ////////
  inline comple& operator()(const long&, const long&) const;
  inline friend std::ostream& operator<<(std::ostream&, const zgematrix&);
  inline void write(const char*) const;
  
  //////// misc ////////
  inline void nullify() const;
  inline void destroy() const;
  
  //////// calc ////////
  inline friend _zgematrix t(const _zgematrix&);
  inline friend _zgematrix i(const _zgematrix&);
  inline friend _zgematrix conj(const _zgematrix&);
  inline friend _zgematrix conjt(const _zgematrix&);
  inline friend void idamax(long&, long&, const _zgematrix&);
  inline friend comple damax(const _zgematrix&);
  
  ///////////////////////////////////////////////
  ///////////// numerical operators /////////////
  ///////////////////////////////////////////////
  //////// unary ////////
  inline friend const _zgematrix& operator+(const _zgematrix&);
  inline friend _zgematrix operator-(const _zgematrix&);
  
  //////// + ////////
  inline friend _zgematrix operator+(const _zgematrix&, const  zgematrix&);
  inline friend _zgematrix operator+(const _zgematrix&, const _zgematrix&);
  inline friend _zgematrix operator+(const _zgematrix&, const  zhematrix&);
  inline friend _zgematrix operator+(const _zgematrix&, const _zhematrix&);
  inline friend _zgematrix operator+(const _zgematrix&, const  zgbmatrix&);
  inline friend _zgematrix operator+(const _zgematrix&, const _zgbmatrix&);
  inline friend _zgematrix operator+(const _zgematrix&, const  zgsmatrix&);
  inline friend _zgematrix operator+(const _zgematrix&, const _zgsmatrix&);
  inline friend _zgematrix operator+(const _zgematrix&, const  zhsmatrix&);
  inline friend _zgematrix operator+(const _zgematrix&, const _zhsmatrix&);
  
  //////// - ////////
  inline friend _zgematrix operator-(const _zgematrix&, const  zgematrix&);
  inline friend _zgematrix operator-(const _zgematrix&, const _zgematrix&);
  inline friend _zgematrix operator-(const _zgematrix&, const  zhematrix&);
  inline friend _zgematrix operator-(const _zgematrix&, const _zhematrix&);
  inline friend _zgematrix operator-(const _zgematrix&, const  zgbmatrix&);
  inline friend _zgematrix operator-(const _zgematrix&, const _zgbmatrix&);
  inline friend _zgematrix operator-(const _zgematrix&, const  zgsmatrix&);
  inline friend _zgematrix operator-(const _zgematrix&, const _zgsmatrix&);
  inline friend _zgematrix operator-(const _zgematrix&, const  zhsmatrix&);
  inline friend _zgematrix operator-(const _zgematrix&, const _zhsmatrix&);
  
  //////// * ////////
  inline friend _zcovector operator*(const _zgematrix&, const  zcovector&);
  inline friend _zcovector operator*(const _zgematrix&, const _zcovector&);
  inline friend _zgematrix operator*(const _zgematrix&, const  zgematrix&);
  inline friend _zgematrix operator*(const _zgematrix&, const _zgematrix&);
  inline friend _zgematrix operator*(const _zgematrix&, const  zhematrix&);
  inline friend _zgematrix operator*(const _zgematrix&, const _zhematrix&);
  inline friend _zgematrix operator*(const _zgematrix&, const  zgbmatrix&);
  inline friend _zgematrix operator*(const _zgematrix&, const _zgbmatrix&);
  inline friend _zgematrix operator*(const _zgematrix&, const  zgsmatrix&);
  inline friend _zgematrix operator*(const _zgematrix&, const _zgsmatrix&);
  inline friend _zgematrix operator*(const _zgematrix&, const  zhsmatrix&);
  inline friend _zgematrix operator*(const _zgematrix&, const _zhsmatrix&);
  inline friend _zgematrix operator*(const _zgematrix&, const     double&);
  inline friend _zgematrix operator*(const _zgematrix&, const     comple&);
  
  //////// / ////////
  inline friend _zgematrix operator/(const _zgematrix&, const     double&);
  inline friend _zgematrix operator/(const _zgematrix&, const     comple&);
  
  //////// double, complex ////////
  inline friend _zgematrix operator*(const     double&, const _zgematrix&);
  inline friend _zgematrix operator*(const     comple&, const _zgematrix&);
};
