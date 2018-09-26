//=============================================================================
//! Complex Double-precision Hermitian Sparse Matrix Class
class zhsmatrix
{
public:
  ///////////////////////////////////////////////
  /////////////////// objects ///////////////////
  ///////////////////////////////////////////////
  long const& m; //!< matrix row size
  long n; //!< matrix column size
  std::vector<zcomponent> data; //!< matrix data
  std::vector< std::vector<uint32_t> > line; //!< vector of vector to store the entry information of component for each row and column
  
  ///////////////////////////////////////////////
  ///////////////// constructors ////////////////
  ///////////////////////////////////////////////
  inline zhsmatrix();
  inline zhsmatrix(const zhsmatrix&);
  inline zhsmatrix(const _zhsmatrix&);
  inline zhsmatrix(const long&, const long=0);
  inline zhsmatrix(const char*);
  inline zhsmatrix(const zgematrix&);
  inline ~zhsmatrix(); //destructor
  
  ///////////////////////////////////////////////
  ////////////////// functions //////////////////
  ///////////////////////////////////////////////
  //////// cast ////////
  inline _zgematrix to_zgematrix() const;
  inline _zhematrix to_zhematrix() const;
  inline _zgsmatrix to_zgsmatrix() const;
  
  //////// io ////////
  inline comple operator()(const long&, const long&) const;
  inline zhecomplex operator()(const long&, const long&);
  inline zhsmatrix& put(const long&, const long&, const comple&);
  inline zhsmatrix& del(const long, const long); //<-- NOT (const long&)
  inline zhsmatrix& del(const long); //<-- NOT (const long&)
  inline friend std::ostream& operator<<(std::ostream&, const zhsmatrix&);
  inline void write(const char*) const;
  inline void read(const char*);

  //////// misc ////////
  inline void clear();
  inline zhsmatrix& zero();
  inline void chsign();
  inline void copy(const zhsmatrix&);
  inline void shallow_copy(const _zhsmatrix&);
  inline zhsmatrix& resize(const long&, const long=0, const long=0);
  inline void stretch(const long&);
  inline void expand(const long&);
  inline bool isListed(const long&, const long&) const;
  inline long number(const long&, const long&) const;
  //inline void unique();
  inline void checkup();
  inline _zrovector row(const long&) const;
  inline _zcovector col(const long&) const;
  inline void diet(const double=DBL_MIN);
  inline friend void swap(zhsmatrix&, zhsmatrix&);
  inline friend _zhsmatrix _(zhsmatrix&);
  
  //////// calc ////////
  inline friend _zhsmatrix t(const zhsmatrix&);
  inline friend void idamax(long&, long&, const zhsmatrix&);
  inline friend comple damax(const zhsmatrix&);
  
  ///////////////////////////////////////////////
  ///////////// numerical operators /////////////
  ///////////////////////////////////////////////
  //////// = ////////
  inline zhsmatrix& operator=(const  zhsmatrix&);
  inline zhsmatrix& operator=(const _zhsmatrix&);
  
  //////// += ////////
  inline zhsmatrix& operator+=(const  zhsmatrix&);
  inline zhsmatrix& operator+=(const _zhsmatrix&);
  
  //////// -= ////////
  inline zhsmatrix& operator-=(const  zhsmatrix&);
  inline zhsmatrix& operator-=(const _zhsmatrix&);
  
  //////// *= ////////
  inline zhsmatrix& operator*=(const     double&);
  
  //////// /= ////////
  inline zhsmatrix& operator/=(const     double&);
  
  //////// unary ////////
  inline friend const zhsmatrix& operator+(const zhsmatrix&);
  inline friend _zhsmatrix operator-(const  zhsmatrix&);
  
  //////// + ////////
  inline friend _zgematrix operator+(const  zhsmatrix&, const  zgematrix&);
  inline friend _zgematrix operator+(const  zhsmatrix&, const _zgematrix&);
  inline friend _zgematrix operator+(const  zhsmatrix&, const  zhematrix&);
  inline friend _zgematrix operator+(const  zhsmatrix&, const _zhematrix&);
  inline friend _zgematrix operator+(const  zhsmatrix&, const  zgbmatrix&);
  inline friend _zgematrix operator+(const  zhsmatrix&, const _zgbmatrix&);
  inline friend _zgsmatrix operator+(const  zhsmatrix&, const  zgsmatrix&);
  inline friend _zgsmatrix operator+(const  zhsmatrix&, const _zgsmatrix&);
  inline friend _zhsmatrix operator+(const  zhsmatrix&, const  zhsmatrix&);
  inline friend _zhsmatrix operator+(const  zhsmatrix&, const _zhsmatrix&);
  
  //////// - ////////
  inline friend _zgematrix operator-(const  zhsmatrix&, const  zgematrix&);
  inline friend _zgematrix operator-(const  zhsmatrix&, const _zgematrix&);
  inline friend _zgematrix operator-(const  zhsmatrix&, const  zhematrix&);
  inline friend _zgematrix operator-(const  zhsmatrix&, const _zhematrix&);
  inline friend _zgematrix operator-(const  zhsmatrix&, const  zgbmatrix&);
  inline friend _zgematrix operator-(const  zhsmatrix&, const _zgbmatrix&);
  inline friend _zgsmatrix operator-(const  zhsmatrix&, const  zgsmatrix&);
  inline friend _zgsmatrix operator-(const  zhsmatrix&, const _zgsmatrix&);
  inline friend _zhsmatrix operator-(const  zhsmatrix&, const  zhsmatrix&);
  inline friend _zhsmatrix operator-(const  zhsmatrix&, const _zhsmatrix&);
  
  //////// * ////////
  inline friend _zcovector operator*(const  zhsmatrix&, const  zcovector&);
  inline friend _zcovector operator*(const  zhsmatrix&, const _zcovector&);
  inline friend _zgematrix operator*(const  zhsmatrix&, const  zgematrix&);
  inline friend _zgematrix operator*(const  zhsmatrix&, const _zgematrix&);  
  inline friend _zgematrix operator*(const  zhsmatrix&, const  zhematrix&);
  inline friend _zgematrix operator*(const  zhsmatrix&, const _zhematrix&);
  inline friend _zgematrix operator*(const  zhsmatrix&, const  zgbmatrix&);
  inline friend _zgematrix operator*(const  zhsmatrix&, const _zgbmatrix&);  
  inline friend _zgsmatrix operator*(const  zhsmatrix&, const  zgsmatrix&);
  inline friend _zgsmatrix operator*(const  zhsmatrix&, const _zgsmatrix&);
  inline friend _zgsmatrix operator*(const  zhsmatrix&, const  zhsmatrix&);
  inline friend _zgsmatrix operator*(const  zhsmatrix&, const _zhsmatrix&);
  inline friend _zhsmatrix operator*(const  zhsmatrix&, const     double&);
  inline friend _zgsmatrix operator*(const  zhsmatrix&, const     comple&);
  
  //////// / ////////
  inline friend _zhsmatrix operator/(const  zhsmatrix&, const     double&);
  inline friend _zgsmatrix operator/(const  zhsmatrix&, const     comple&);
  
  //////// double, comple ////////
  inline friend _zhsmatrix operator*(const     double&, const  zhsmatrix&);
  inline friend _zgsmatrix operator*(const     comple&, const  zhsmatrix&);
};
