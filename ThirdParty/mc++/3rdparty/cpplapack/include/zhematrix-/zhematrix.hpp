//=============================================================================
//! Complex Double-precision Hermitian Matrix Class [l-type (UPLO=l) Strage]
/*! The imaginary part of every diagonal component is not referenced. */
class zhematrix
{
public:
  ///////////////////////////////////////////////
  /////////////////// objects ///////////////////
  ///////////////////////////////////////////////
  long const& m; //!< matrix row size
  long n; //!< matrix column size
  comple* array; //!< 1D array to store matrix data
  comple** darray; //!< array of pointers of column head addresses
  
  ///////////////////////////////////////////////
  ///////////////// constructors ////////////////
  ///////////////////////////////////////////////
  inline zhematrix();
  inline zhematrix(const zhematrix&);
  inline zhematrix(const _zhematrix&);
  inline zhematrix(const long&);
  inline zhematrix(const char*);
  inline ~zhematrix(); //destructor
  
  ///////////////////////////////////////////////
  ////////////////// functions //////////////////
  ///////////////////////////////////////////////
  //////// cast ////////
  inline _zgematrix to_zgematrix() const;
  
  //////// io ////////
  inline zhecomplex operator()(const long&, const long&);
  inline comple  operator()(const long&, const long&) const;
  inline zhematrix& set(const long&, const long&, const comple&);
  inline friend std::ostream& operator<<(std::ostream&, const zhematrix&);
  inline void write(const char*) const;
  inline void read(const char*);
  
  //////// misc ////////
  inline void complete() const;
  inline void clear();
  inline zhematrix& zero();
  inline zhematrix& identity();
  inline void chsign();
  inline void copy(const zhematrix&);
  inline void shallow_copy(const _zhematrix&);
  inline void resize(const long&);
  inline _zrovector row(const long&) const;
  inline _zcovector col(const long&) const;
  inline friend void swap(zhematrix&, zhematrix&);
  inline friend _zhematrix _(zhematrix&);
  
  //////// calc ////////
  inline friend _zhematrix t(const zhematrix&);
  inline friend _zgematrix i(const zhematrix&);
  inline friend _zhematrix conj(const zhematrix&);
  inline friend _zhematrix conjt(const zhematrix&);
  
  //////// lapack ////////
  inline long zhesv(zgematrix&);
  inline long zhesv(zcovector&);
  inline long zheev(std::vector<double>&, const bool&);
  inline long zheev(std::vector<double>&, std::vector<zcovector>&);
  inline long zheev(std::vector<double>&, std::vector<zrovector>&);
  
  ///////////////////////////////////////////////
  ///////////// numerical operators /////////////
  ///////////////////////////////////////////////
  //////// = ////////
  inline zhematrix& operator=(const  zhematrix&);
  inline zhematrix& operator=(const _zhematrix&);
  
  //////// += ////////
  inline zhematrix& operator+=(const  zhematrix&);
  inline zhematrix& operator+=(const _zhematrix&);
  
  //////// -= ////////
  inline zhematrix& operator-=(const  zhematrix&);
  inline zhematrix& operator-=(const _zhematrix&);
  
  //////// *= ////////
  inline zhematrix& operator*=(const  zhematrix&);
  inline zhematrix& operator*=(const _zhematrix&);
  inline zhematrix& operator*=(const     double&);
  
  //////// /= ////////
  inline zhematrix& operator/=(const     double&);
  
  //////// unary ////////
  inline friend const zhematrix& operator+(const zhematrix&);
  inline friend _zhematrix operator-(const  zhematrix&);
  
  //////// + ////////
  inline friend _zgematrix operator+(const  zhematrix&, const  zgematrix&);
  inline friend _zgematrix operator+(const  zhematrix&, const _zgematrix&);
  inline friend _zhematrix operator+(const  zhematrix&, const  zhematrix&);
  inline friend _zhematrix operator+(const  zhematrix&, const _zhematrix&);
  inline friend _zgematrix operator+(const  zhematrix&, const  zgbmatrix&);
  inline friend _zgematrix operator+(const  zhematrix&, const _zgbmatrix&);
  inline friend _zgematrix operator+(const  zhematrix&, const  zgsmatrix&);
  inline friend _zgematrix operator+(const  zhematrix&, const _zgsmatrix&);
  inline friend _zgematrix operator+(const  zhematrix&, const  zhsmatrix&);
  inline friend _zgematrix operator+(const  zhematrix&, const _zhsmatrix&);

  //////// - ////////
  inline friend _zgematrix operator-(const  zhematrix&, const  zgematrix&);
  inline friend _zgematrix operator-(const  zhematrix&, const _zgematrix&);
  inline friend _zhematrix operator-(const  zhematrix&, const  zhematrix&);
  inline friend _zhematrix operator-(const  zhematrix&, const _zhematrix&);
  inline friend _zgematrix operator-(const  zhematrix&, const  zgbmatrix&);
  inline friend _zgematrix operator-(const  zhematrix&, const _zgbmatrix&);
  inline friend _zgematrix operator-(const  zhematrix&, const  zgsmatrix&);
  inline friend _zgematrix operator-(const  zhematrix&, const _zgsmatrix&);
  inline friend _zgematrix operator-(const  zhematrix&, const  zhsmatrix&);
  inline friend _zgematrix operator-(const  zhematrix&, const _zhsmatrix&);
  
  //////// * ////////
  inline friend _zcovector operator*(const  zhematrix&, const  zcovector&);
  inline friend _zcovector operator*(const  zhematrix&, const _zcovector&);
  inline friend _zgematrix operator*(const  zhematrix&, const  zgematrix&);
  inline friend _zgematrix operator*(const  zhematrix&, const _zgematrix&);
  inline friend _zgematrix operator*(const  zhematrix&, const  zhematrix&);
  inline friend _zgematrix operator*(const  zhematrix&, const _zhematrix&);
  inline friend _zgematrix operator*(const  zhematrix&, const  zgbmatrix&);
  inline friend _zgematrix operator*(const  zhematrix&, const _zgbmatrix&);
  inline friend _zgematrix operator*(const  zhematrix&, const  zgsmatrix&);
  inline friend _zgematrix operator*(const  zhematrix&, const _zgsmatrix&);
  inline friend _zgematrix operator*(const  zhematrix&, const  zhsmatrix&);
  inline friend _zgematrix operator*(const  zhematrix&, const _zhsmatrix&);
  inline friend _zhematrix operator*(const  zhematrix&, const     double&);
  inline friend _zgematrix operator*(const  zhematrix&, const     comple&);
  
  //////// / ////////
  inline friend _zhematrix operator/(const  zhematrix&, const     double&);
  inline friend _zgematrix operator/(const  zhematrix&, const     comple&);
  
  //////// double, comple ////////
  inline friend _zhematrix operator*(const     double&, const  zhematrix&);
  inline friend _zgematrix operator*(const     comple&, const  zhematrix&);
};
