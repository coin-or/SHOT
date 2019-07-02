//=============================================================================
//! (DO NOT USE) Smart-temporary Complex Double-precision Hermitian Sparse Matrix Class
class _zhsmatrix
{
public:
  ///////////////////////////////////////////////
  /////////////////// objects ///////////////////
  ///////////////////////////////////////////////
  long const& m; //!< matrix row size
  //mutable long const& m; //!< matrix row size
  mutable long n; //!< matrix column size
  mutable std::vector<zcomponent> data; //!< matrix data
  mutable std::vector< std::vector<uint32_t> > line; //!< vector of vector to store the entry information of component for each row and column
  
  ///////////////////////////////////////////////
  ///////////////// constructors ////////////////
  ///////////////////////////////////////////////
  inline _zhsmatrix();
  inline _zhsmatrix(const _zhsmatrix&);
  inline ~_zhsmatrix(); //destructor
  
  ///////////////////////////////////////////////
  ////////////////// functions //////////////////
  ///////////////////////////////////////////////
  //////// cast ////////
  inline _zgematrix to_zgematrix() const;
  inline _zhematrix to_zhematrix() const;
  inline _zgsmatrix to_zgsmatrix() const;
  
  //////// io ////////
  inline comple operator()(const long&, const long&) const;//not return zhecomplex
  inline friend std::ostream& operator<<(std::ostream&, const _zhsmatrix&);
  inline void write(const char*) const;

  //////// misc ////////
  inline void nullify() const;
  inline void destroy() const;
  
  //////// calc ////////
  inline friend _zhsmatrix t(const zhsmatrix&);
  inline friend void idamax(long&, long&, const zhsmatrix&);
  inline friend comple damax(const zhsmatrix&);
  
  ///////////////////////////////////////////////
  ///////////// numerical operators /////////////
  ///////////////////////////////////////////////
  //////// unary ////////
  inline friend const _zhsmatrix& operator+(const _zhsmatrix&);
  inline friend _zhsmatrix operator-(const _zhsmatrix&);
  
  //////// + ////////
  inline friend _zgematrix operator+(const _zhsmatrix&, const  zgematrix&);
  inline friend _zgematrix operator+(const _zhsmatrix&, const _zgematrix&);
  inline friend _zgematrix operator+(const _zhsmatrix&, const  zhematrix&);
  inline friend _zgematrix operator+(const _zhsmatrix&, const _zhematrix&);
  inline friend _zgematrix operator+(const _zhsmatrix&, const  zgbmatrix&);
  inline friend _zgematrix operator+(const _zhsmatrix&, const _zgbmatrix&);
  inline friend _zgsmatrix operator+(const _zhsmatrix&, const  zgsmatrix&);
  inline friend _zgsmatrix operator+(const _zhsmatrix&, const _zgsmatrix&);
  inline friend _zhsmatrix operator+(const _zhsmatrix&, const  zhsmatrix&);
  inline friend _zhsmatrix operator+(const _zhsmatrix&, const _zhsmatrix&);
  
  //////// - ////////
  inline friend _zgematrix operator-(const _zhsmatrix&, const  zgematrix&);
  inline friend _zgematrix operator-(const _zhsmatrix&, const _zgematrix&);
  inline friend _zgematrix operator-(const _zhsmatrix&, const  zhematrix&);
  inline friend _zgematrix operator-(const _zhsmatrix&, const _zhematrix&);
  inline friend _zgematrix operator-(const _zhsmatrix&, const  zgbmatrix&);
  inline friend _zgematrix operator-(const _zhsmatrix&, const _zgbmatrix&);
  inline friend _zgsmatrix operator-(const _zhsmatrix&, const  zgsmatrix&);
  inline friend _zgsmatrix operator-(const _zhsmatrix&, const _zgsmatrix&);
  inline friend _zhsmatrix operator-(const _zhsmatrix&, const  zhsmatrix&);
  inline friend _zhsmatrix operator-(const _zhsmatrix&, const _zhsmatrix&);
  
  //////// * ////////
  inline friend _zcovector operator*(const _zhsmatrix&, const  zcovector&);
  inline friend _zcovector operator*(const _zhsmatrix&, const _zcovector&);
  inline friend _zgematrix operator*(const _zhsmatrix&, const  zgematrix&);
  inline friend _zgematrix operator*(const _zhsmatrix&, const _zgematrix&);
  inline friend _zgematrix operator*(const _zhsmatrix&, const  zhematrix&);
  inline friend _zgematrix operator*(const _zhsmatrix&, const _zhematrix&);
  inline friend _zgematrix operator*(const _zhsmatrix&, const  zgbmatrix&);
  inline friend _zgematrix operator*(const _zhsmatrix&, const _zgbmatrix&);
  inline friend _zgsmatrix operator*(const _zhsmatrix&, const  zgsmatrix&);
  inline friend _zgsmatrix operator*(const _zhsmatrix&, const _zgsmatrix&);
  inline friend _zgsmatrix operator*(const _zhsmatrix&, const  zhsmatrix&);
  inline friend _zgsmatrix operator*(const _zhsmatrix&, const _zhsmatrix&);
  inline friend _zhsmatrix operator*(const _zhsmatrix&, const     double&);
  inline friend _zgsmatrix operator*(const _zhsmatrix&, const     comple&);
  
  //////// / ////////
  inline friend _zhsmatrix operator/(const _zhsmatrix&, const     double&);
  inline friend _zgsmatrix operator/(const _zhsmatrix&, const     comple&);
  
  //////// double, complex ////////
  inline friend _zhsmatrix operator*(const     double&, const _zhsmatrix&);
  inline friend _zgsmatrix operator*(const     comple&, const _zhsmatrix&);
};
