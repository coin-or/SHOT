//=============================================================================
//! Complex Double-precision General Band Matrix Class
class zgbmatrix
{
public:
  ///////////////////////////////////////////////
  /////////////////// objects ///////////////////
  ///////////////////////////////////////////////
  long m; //!< matrix row size
  long n; //!< matrix column size
  long kl; //!< lower band width
  long ku; //!< upper band width
  comple* array; //!< 1D array to store matrix data
  comple** darray; //!< array of pointers of column head addresses
  
  ///////////////////////////////////////////////
  ///////////////// constructors ////////////////
  ///////////////////////////////////////////////
  inline zgbmatrix();
  inline zgbmatrix(const zgbmatrix&);
  inline zgbmatrix(const _zgbmatrix&);
  inline zgbmatrix(const long&, const long&, const long&, const long&);
  inline zgbmatrix(const char*);
  inline ~zgbmatrix(); //destructor
  
  ///////////////////////////////////////////////
  ////////////////// functions //////////////////
  ///////////////////////////////////////////////
  //////// cast ////////
  inline _zgematrix to_zgematrix() const;
  
  //////// io ////////
  inline comple& operator()(const long&, const long&);
  inline comple operator()(const long&, const long&) const;
  inline zgbmatrix& set(const long&, const long&, const comple&);
  inline friend std::ostream& operator<<(std::ostream&, const zgbmatrix&);
  inline void write(const char*) const;
  inline void read(const char*);
  
  //////// misc ////////
  inline void clear();
  inline zgbmatrix& zero();
  inline zgbmatrix& identity();
  inline void chsign();
  inline void copy(const zgbmatrix&);
  inline void shallow_copy(const _zgbmatrix&);
  inline void resize(const long&, const long&, const long&, const long&);
  inline _zrovector row(const long&) const;
  inline _zcovector col(const long&) const;
  inline friend void swap(zgbmatrix&, zgbmatrix&);
  inline friend _zgbmatrix _(zgbmatrix&);
  
  //////// calc ////////
  inline friend _zgbmatrix t(const zgbmatrix&);
  inline friend _zgematrix i(const zgbmatrix&);
  inline friend _zgbmatrix conj(const zgbmatrix&);
  inline friend _zgbmatrix conjt(const zgbmatrix&);
  
  //////// lapack ////////
  inline long zgbsv(zgematrix&);
  inline long zgbsv(zcovector&);
  
  ///////////////////////////////////////////////
  ///////////// numerical operators /////////////
  ///////////////////////////////////////////////
  //////// = ////////
  inline zgbmatrix& operator=(const  zgbmatrix&);
  inline zgbmatrix& operator=(const _zgbmatrix&);
  
  //////// += ////////
  inline zgbmatrix& operator+=(const  zgbmatrix&);
  inline zgbmatrix& operator+=(const _zgbmatrix&);
  
  //////// -= ////////
  inline zgbmatrix& operator-=(const  zgbmatrix&);
  inline zgbmatrix& operator-=(const _zgbmatrix&);
  
  //////// *= ////////
  inline zgbmatrix& operator*=(const  zgbmatrix&);
  inline zgbmatrix& operator*=(const _zgbmatrix&);
  inline zgbmatrix& operator*=(const     double&);
  inline zgbmatrix& operator*=(const     comple&);
  
  //////// /= ////////
  inline zgbmatrix& operator/=(const     double&);  
  inline zgbmatrix& operator/=(const     comple&);
  
  //////// unary ////////
  inline friend const zgbmatrix& operator+(const zgbmatrix&);
  inline friend _zgbmatrix operator-(const  zgbmatrix&);
  
  //////// + ////////
  inline friend _zgematrix operator+(const  zgbmatrix&, const  zgematrix&);
  inline friend _zgematrix operator+(const  zgbmatrix&, const _zgematrix&);
  inline friend _zgematrix operator+(const  zgbmatrix&, const  zhematrix&);
  inline friend _zgematrix operator+(const  zgbmatrix&, const _zhematrix&);
  inline friend _zgbmatrix operator+(const  zgbmatrix&, const  zgbmatrix&);
  inline friend _zgbmatrix operator+(const  zgbmatrix&, const _zgbmatrix&);
  inline friend _zgematrix operator+(const  zgbmatrix&, const  zgsmatrix&);
  inline friend _zgematrix operator+(const  zgbmatrix&, const _zgsmatrix&);
  inline friend _zgematrix operator+(const  zgbmatrix&, const  zhsmatrix&);
  inline friend _zgematrix operator+(const  zgbmatrix&, const _zhsmatrix&);
  
  //////// - ////////
  inline friend _zgematrix operator-(const  zgbmatrix&, const  zgematrix&);
  inline friend _zgematrix operator-(const  zgbmatrix&, const _zgematrix&);
  inline friend _zgematrix operator-(const  zgbmatrix&, const  zhematrix&);
  inline friend _zgematrix operator-(const  zgbmatrix&, const _zhematrix&);
  inline friend _zgbmatrix operator-(const  zgbmatrix&, const  zgbmatrix&);
  inline friend _zgbmatrix operator-(const  zgbmatrix&, const _zgbmatrix&);
  inline friend _zgematrix operator-(const  zgbmatrix&, const  zgsmatrix&);
  inline friend _zgematrix operator-(const  zgbmatrix&, const _zgsmatrix&);
  inline friend _zgematrix operator-(const  zgbmatrix&, const  zhsmatrix&);
  inline friend _zgematrix operator-(const  zgbmatrix&, const _zhsmatrix&);
  
  //////// * ////////
  inline friend _zcovector operator*(const  zgbmatrix&, const  zcovector&);
  inline friend _zcovector operator*(const  zgbmatrix&, const _zcovector&);
  inline friend _zgematrix operator*(const  zgbmatrix&, const  zgematrix&);
  inline friend _zgematrix operator*(const  zgbmatrix&, const _zgematrix&);
  inline friend _zgematrix operator*(const  zgbmatrix&, const  zhematrix&);
  inline friend _zgematrix operator*(const  zgbmatrix&, const _zhematrix&);
  inline friend _zgbmatrix operator*(const  zgbmatrix&, const  zgbmatrix&);
  inline friend _zgbmatrix operator*(const  zgbmatrix&, const _zgbmatrix&);
  inline friend _zgematrix operator*(const  zgbmatrix&, const  zgsmatrix&);
  inline friend _zgematrix operator*(const  zgbmatrix&, const _zgsmatrix&);
  inline friend _zgematrix operator*(const  zgbmatrix&, const  zhsmatrix&);
  inline friend _zgematrix operator*(const  zgbmatrix&, const _zhsmatrix&);
  inline friend _zgbmatrix operator*(const  zgbmatrix&, const     double&);
  inline friend _zgbmatrix operator*(const  zgbmatrix&, const     comple&);
  
  //////// / ////////
  inline friend _zgbmatrix operator/(const  zgbmatrix&, const     double&);
  inline friend _zgbmatrix operator/(const  zgbmatrix&, const     comple&);
  
  //////// double, comple ////////
  inline friend _zgbmatrix operator*(const     double&, const  zgbmatrix&);
  inline friend _zgbmatrix operator*(const     comple&, const  zgbmatrix&);
};
