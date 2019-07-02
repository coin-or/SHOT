//=============================================================================
//! (DO NOT USE) Smart-temporary Real Double-precision General Sparse Matrix Class
class _zgsmatrix
{
public:
  ///////////////////////////////////////////////
  /////////////////// objects ///////////////////
  ///////////////////////////////////////////////
  mutable long m; //!< matrix row size
  mutable long n; //!< matrix column size
  mutable std::vector<zcomponent> data; //!< matrix data
  mutable std::vector< std::vector<uint32_t> > rows; //!< array of vector to store the entry information of component for each row
  mutable std::vector< std::vector<uint32_t> > cols; //!< array of vector to store the entry information of component for each column
  
  ///////////////////////////////////////////////
  ///////////////// constructors ////////////////
  ///////////////////////////////////////////////
  inline _zgsmatrix();
  inline _zgsmatrix(const _zgsmatrix&);
  inline ~_zgsmatrix(); //destructor
  
  ///////////////////////////////////////////////
  ////////////////// functions //////////////////
  ///////////////////////////////////////////////
  //////// cast ////////
  inline _zgematrix to_zgematrix() const;
  
  //////// io ////////
  inline comple operator()(const long&, const long&) const;//not return comple&
  inline friend std::ostream& operator<<(std::ostream&, const _zgsmatrix&);
  inline void write(const char*) const;
  
  //////// misc ////////
  inline void nullify() const;
  inline void destroy() const;
  
  //////// calc ////////
  inline friend _zgsmatrix t(const zgsmatrix&);
  inline friend void idamax(long&, long&, const zgsmatrix&);
  inline friend comple damax(const zgsmatrix&);
  
  ///////////////////////////////////////////////
  ///////////// numerical operators /////////////
  ///////////////////////////////////////////////
  //////// unary ////////
  inline friend const _zgsmatrix& operator+(const _zgsmatrix&);
  inline friend _zgsmatrix operator-(const _zgsmatrix&);
  
  //////// + ////////
  inline friend _zgematrix operator+(const _zgsmatrix&, const  zgematrix&);
  inline friend _zgematrix operator+(const _zgsmatrix&, const _zgematrix&);
  inline friend _zgematrix operator+(const _zgsmatrix&, const  zhematrix&);
  inline friend _zgematrix operator+(const _zgsmatrix&, const _zhematrix&);
  inline friend _zgematrix operator+(const _zgsmatrix&, const  zgbmatrix&);
  inline friend _zgematrix operator+(const _zgsmatrix&, const _zgbmatrix&);
  inline friend _zgsmatrix operator+(const _zgsmatrix&, const  zgsmatrix&);
  inline friend _zgsmatrix operator+(const _zgsmatrix&, const _zgsmatrix&);
  inline friend _zgsmatrix operator+(const _zgsmatrix&, const  zhsmatrix&);
  inline friend _zgsmatrix operator+(const _zgsmatrix&, const _zhsmatrix&);
  
  //////// - ////////
  inline friend _zgematrix operator-(const _zgsmatrix&, const  zgematrix&);
  inline friend _zgematrix operator-(const _zgsmatrix&, const _zgematrix&);
  inline friend _zgematrix operator-(const _zgsmatrix&, const  zhematrix&);
  inline friend _zgematrix operator-(const _zgsmatrix&, const _zhematrix&);
  inline friend _zgematrix operator-(const _zgsmatrix&, const  zgbmatrix&);
  inline friend _zgematrix operator-(const _zgsmatrix&, const _zgbmatrix&);
  inline friend _zgsmatrix operator-(const _zgsmatrix&, const  zgsmatrix&);
  inline friend _zgsmatrix operator-(const _zgsmatrix&, const _zgsmatrix&);
  inline friend _zgsmatrix operator-(const _zgsmatrix&, const  zhsmatrix&);
  inline friend _zgsmatrix operator-(const _zgsmatrix&, const _zhsmatrix&);
  
  //////// * ////////
  inline friend _zcovector operator*(const _zgsmatrix&, const  zcovector&);
  inline friend _zcovector operator*(const _zgsmatrix&, const _zcovector&);
  inline friend _zgematrix operator*(const _zgsmatrix&, const  zgematrix&);
  inline friend _zgematrix operator*(const _zgsmatrix&, const _zgematrix&);
  inline friend _zgematrix operator*(const _zgsmatrix&, const  zhematrix&);
  inline friend _zgematrix operator*(const _zgsmatrix&, const _zhematrix&);
  inline friend _zgematrix operator*(const _zgsmatrix&, const  zgbmatrix&);
  inline friend _zgematrix operator*(const _zgsmatrix&, const _zgbmatrix&);
  inline friend _zgsmatrix operator*(const _zgsmatrix&, const  zgsmatrix&);
  inline friend _zgsmatrix operator*(const _zgsmatrix&, const _zgsmatrix&);
  inline friend _zgsmatrix operator*(const _zgsmatrix&, const  zhsmatrix&);
  inline friend _zgsmatrix operator*(const _zgsmatrix&, const _zhsmatrix&);
  inline friend _zgsmatrix operator*(const _zgsmatrix&, const     double&);
  inline friend _zgsmatrix operator*(const _zgsmatrix&, const     comple&);
  
  //////// / ////////
  inline friend _zgsmatrix operator/(const _zgsmatrix&, const     double&);
  inline friend _zgsmatrix operator/(const _zgsmatrix&, const     comple&);
  
  //////// double, complex ////////
  inline friend _zgsmatrix operator*(const     double&, const _zgsmatrix&);
  inline friend _zgsmatrix operator*(const     comple&, const _zgsmatrix&);
};
