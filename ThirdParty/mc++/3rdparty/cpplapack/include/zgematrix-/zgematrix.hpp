//=============================================================================
//! Complex Double-precision General Dence Matrix Class
class zgematrix
{
public:
  ///////////////////////////////////////////////
  /////////////////// objects ///////////////////
  ///////////////////////////////////////////////
  long m; //!< matrix row size
  long n; //!< matrix column size
  comple* array; //!< 1D array to store matrix data
  comple** darray; //!< array of pointers of column head addresses
  
  ///////////////////////////////////////////////
  ///////////////// constructors ////////////////
  ///////////////////////////////////////////////
  inline zgematrix();
  inline zgematrix(const zgematrix&);
  inline zgematrix(const _zgematrix&);
  inline zgematrix(const long&, const long&);
  inline zgematrix(const char*);
  inline ~zgematrix(); //destructor
  
  ///////////////////////////////////////////////
  ////////////////// functions //////////////////
  ///////////////////////////////////////////////
  //////// cast ////////
  
  //////// io ////////
  inline comple& operator()(const long&, const long&);
  inline comple operator()(const long&, const long&) const;
  inline zgematrix& set(const long&, const long&, const comple&);
  inline friend std::ostream& operator<<(std::ostream&, const zgematrix&);
  inline void write(const char*) const;
  inline void read(const char*);

  //////// misc ////////
  inline void clear();
  inline zgematrix& zero();
  inline zgematrix& identity();
  inline void chsign();
  inline void copy(const zgematrix&);
  inline void shallow_copy(const _zgematrix&);
  inline void resize(const long&, const long&);
  inline _zrovector row(const long&) const;
  inline _zcovector col(const long&) const;
  inline friend void swap(zgematrix&, zgematrix&);
  inline friend _zgematrix _(zgematrix&);
  
  //////// calc ////////
  inline friend _zgematrix t(const zgematrix&);
  inline friend _zgematrix i(const zgematrix&);
  inline friend _zgematrix conj(const zgematrix&);
  inline friend _zgematrix conjt(const zgematrix&);
  inline friend void idamax(long&, long&, const zgematrix&);
  inline friend comple damax(const zgematrix&);
  
  //////// lapack ////////
  inline long zgesv(zgematrix&);
  inline long zgesv(zcovector&);
  inline long zgels(zgematrix&);
  inline long zgels(zcovector&);
  inline long zgels(zgematrix&, drovector&);
  inline long zgels(zcovector&, double&);
  inline long zgelss(zcovector&, dcovector&, long&, const double);
  inline long zgelss(zgematrix&, dcovector&, long&, const double);
  inline long zgeev(std::vector< comple >&);
  inline long zgeev(std::vector< comple >&, std::vector<zcovector>&);
  inline long zgeev(std::vector< comple >&, std::vector<zrovector>&);
  //inline long zgegv()
  inline long zgesvd(dcovector&, zgematrix&, zgematrix&);
  
  ///////////////////////////////////////////////
  ///////////// numerical operators /////////////
  ///////////////////////////////////////////////
  //////// = ////////
  inline zgematrix& operator=(const  zgematrix&);
  inline zgematrix& operator=(const _zgematrix&);
  
  //////// += ////////
  inline zgematrix& operator+=(const  zgematrix&);
  inline zgematrix& operator+=(const _zgematrix&);
  inline zgematrix& operator+=(const  zhematrix&);
  inline zgematrix& operator+=(const _zhematrix&);
  inline zgematrix& operator+=(const  zgbmatrix&);
  inline zgematrix& operator+=(const _zgbmatrix&);
  inline zgematrix& operator+=(const  zgsmatrix&);
  inline zgematrix& operator+=(const _zgsmatrix&);
  inline zgematrix& operator+=(const  zhsmatrix&);
  inline zgematrix& operator+=(const _zhsmatrix&);
  
  //////// -= ////////
  inline zgematrix& operator-=(const  zgematrix&);
  inline zgematrix& operator-=(const _zgematrix&);
  inline zgematrix& operator-=(const  zhematrix&);
  inline zgematrix& operator-=(const _zhematrix&);
  inline zgematrix& operator-=(const  zgbmatrix&);
  inline zgematrix& operator-=(const _zgbmatrix&);
  inline zgematrix& operator-=(const  zgsmatrix&);
  inline zgematrix& operator-=(const _zgsmatrix&);
  inline zgematrix& operator-=(const  zhsmatrix&);
  inline zgematrix& operator-=(const _zhsmatrix&);

  //////// *= ////////
  inline zgematrix& operator*=(const  zgematrix&);
  inline zgematrix& operator*=(const _zgematrix&);
  inline zgematrix& operator*=(const  zhematrix&);
  inline zgematrix& operator*=(const _zhematrix&);
  inline zgematrix& operator*=(const  zgbmatrix&);
  inline zgematrix& operator*=(const _zgbmatrix&);
  inline zgematrix& operator*=(const  zgsmatrix&);
  inline zgematrix& operator*=(const _zgsmatrix&);
  inline zgematrix& operator*=(const  zhsmatrix&);
  inline zgematrix& operator*=(const _zhsmatrix&);
  inline zgematrix& operator*=(const     double&);
  inline zgematrix& operator*=(const     comple&);
  
  //////// /= ////////
  inline zgematrix& operator/=(const     double&);
  inline zgematrix& operator/=(const     comple&);

  //////// unary ////////
  inline friend const zgematrix& operator+(const zgematrix&);
  inline friend _zgematrix operator-(const  zgematrix&);
  
  //////// + ////////
  inline friend _zgematrix operator+(const  zgematrix&, const  zgematrix&);
  inline friend _zgematrix operator+(const  zgematrix&, const _zgematrix&);
  inline friend _zgematrix operator+(const  zgematrix&, const  zhematrix&);
  inline friend _zgematrix operator+(const  zgematrix&, const _zhematrix&);
  inline friend _zgematrix operator+(const  zgematrix&, const  zgbmatrix&);
  inline friend _zgematrix operator+(const  zgematrix&, const _zgbmatrix&);
  inline friend _zgematrix operator+(const  zgematrix&, const  zgsmatrix&);
  inline friend _zgematrix operator+(const  zgematrix&, const _zgsmatrix&);
  inline friend _zgematrix operator+(const  zgematrix&, const  zhsmatrix&);
  inline friend _zgematrix operator+(const  zgematrix&, const _zhsmatrix&);
  
  //////// - ////////
  inline friend _zgematrix operator-(const  zgematrix&, const  zgematrix&);
  inline friend _zgematrix operator-(const  zgematrix&, const _zgematrix&);
  inline friend _zgematrix operator-(const  zgematrix&, const  zhematrix&);
  inline friend _zgematrix operator-(const  zgematrix&, const _zhematrix&);
  inline friend _zgematrix operator-(const  zgematrix&, const  zgbmatrix&);
  inline friend _zgematrix operator-(const  zgematrix&, const _zgbmatrix&);
  inline friend _zgematrix operator-(const  zgematrix&, const  zgsmatrix&);
  inline friend _zgematrix operator-(const  zgematrix&, const _zgsmatrix&);
  inline friend _zgematrix operator-(const  zgematrix&, const  zhsmatrix&);
  inline friend _zgematrix operator-(const  zgematrix&, const _zhsmatrix&);
  
  //////// * ////////
  inline friend _zcovector operator*(const  zgematrix&, const  zcovector&);
  inline friend _zcovector operator*(const  zgematrix&, const _zcovector&);
  inline friend _zgematrix operator*(const  zgematrix&, const  zgematrix&);
  inline friend _zgematrix operator*(const  zgematrix&, const _zgematrix&);
  inline friend _zgematrix operator*(const  zgematrix&, const  zhematrix&);
  inline friend _zgematrix operator*(const  zgematrix&, const _zhematrix&);
  inline friend _zgematrix operator*(const  zgematrix&, const  zgbmatrix&);
  inline friend _zgematrix operator*(const  zgematrix&, const _zgbmatrix&);  
  inline friend _zgematrix operator*(const  zgematrix&, const  zgsmatrix&);
  inline friend _zgematrix operator*(const  zgematrix&, const _zgsmatrix&);
  inline friend _zgematrix operator*(const  zgematrix&, const  zhsmatrix&);
  inline friend _zgematrix operator*(const  zgematrix&, const _zhsmatrix&);
  inline friend _zgematrix operator*(const  zgematrix&, const     double&);
  inline friend _zgematrix operator*(const  zgematrix&, const     comple&);
  
  //////// / ////////
  inline friend _zgematrix operator/(const  zgematrix&, const     double&);
  inline friend _zgematrix operator/(const  zgematrix&, const     comple&);

  //////// double, comple ////////
  inline friend _zgematrix operator*(const     double&, const  zgematrix&);
  inline friend _zgematrix operator*(const     comple&, const  zgematrix&);
};
