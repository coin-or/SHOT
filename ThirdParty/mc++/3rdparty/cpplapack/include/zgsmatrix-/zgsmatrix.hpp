//=============================================================================
//! Complex Double-precision General Sparse Matrix Class
class zgsmatrix
{
public:
  ///////////////////////////////////////////////
  /////////////////// objects ///////////////////
  ///////////////////////////////////////////////
  long m; //!< matrix row size
  long n; //!< matrix column size
  std::vector<zcomponent> data; //!< matrix data
  std::vector< std::vector<uint32_t> > rows; //!< array of vector to store the entry information of component for each row
  std::vector< std::vector<uint32_t> > cols; //!< array of vector to store the entry information of component for each column
  
  ///////////////////////////////////////////////
  ///////////////// constructors ////////////////
  ///////////////////////////////////////////////
  inline zgsmatrix();
  inline zgsmatrix(const zgsmatrix&);
  inline zgsmatrix(const _zgsmatrix&);
  inline zgsmatrix(const long&, const long&, const long=0);
  inline zgsmatrix(const char*);
  inline ~zgsmatrix(); //destructor
  
  ///////////////////////////////////////////////
  ////////////////// functions //////////////////
  ///////////////////////////////////////////////
  //////// cast ////////
  inline _zgematrix to_zgematrix() const;
  
  //////// io ////////
  inline comple operator()(const long&, const long&) const;
  inline comple& operator()(const long&, const long&);
  inline zgsmatrix& put(const long&, const long&, const comple&);
  inline zgsmatrix& del(const long, const long); //<-- NOT (const long&)
  inline zgsmatrix& del(const long); //<-- NOT (const long&)
  inline friend std::ostream& operator<<(std::ostream&, const zgsmatrix&);
  inline void write(const char*) const;
  inline void read(const char*);

  //////// misc ////////
  inline void clear();
  inline zgsmatrix& zero();
  inline void chsign();
  inline void copy(const zgsmatrix&);
  inline void shallow_copy(const _zgsmatrix&);
  inline zgsmatrix& resize(const long&, const long&, const long=0, const long=0);
  inline void stretch(const long&, const long&);
  inline void expand(const long&);
  inline bool isListed(const long&, const long&);
  inline long number(const long&, const long&);
  inline void checkup();
  inline void diet(const double=DBL_MIN);
  inline _zrovector row(const long&) const;
  inline _zcovector col(const long&) const;
  inline friend void swap(zgsmatrix&, zgsmatrix&);
  inline friend _zgsmatrix _(zgsmatrix&);
  
  //////// calc ////////
  inline friend _zgsmatrix t(const zgsmatrix&);
  ////inline friend _zgematrix i(const zgsmatrix&);
  inline friend _zgsmatrix conj(const zgsmatrix&);
  inline friend _zgsmatrix conjt(const zgsmatrix&);
  inline friend void idamax(long&, long&, const zgsmatrix&);
  inline friend comple damax(const zgsmatrix&);
  
  ///////////////////////////////////////////////
  ///////////// numerical operators /////////////
  ///////////////////////////////////////////////
  //////// = ////////
  inline zgsmatrix& operator=(const  zgsmatrix&);
  inline zgsmatrix& operator=(const _zgsmatrix&);
  
  //////// += ////////
  inline zgsmatrix& operator+=(const  zgsmatrix&);
  inline zgsmatrix& operator+=(const _zgsmatrix&);
  
  //////// -= ////////
  inline zgsmatrix& operator-=(const  zgsmatrix&);
  inline zgsmatrix& operator-=(const _zgsmatrix&);
  
  //////// *= ////////
  inline zgsmatrix& operator*=(const  zgsmatrix&);
  inline zgsmatrix& operator*=(const _zgsmatrix&);
  inline zgsmatrix& operator*=(const     double&);
  inline zgsmatrix& operator*=(const     comple&);
  
  //////// /= ////////
  inline zgsmatrix& operator/=(const     double&);
  inline zgsmatrix& operator/=(const     comple&);

  //////// unary ////////
  inline friend const zgsmatrix& operator+(const zgsmatrix&);
  inline friend _zgsmatrix operator-(const  zgsmatrix&);
  
  //////// + ////////
  inline friend _zgematrix operator+(const  zgsmatrix&, const  zgematrix&);
  inline friend _zgematrix operator+(const  zgsmatrix&, const _zgematrix&);
  inline friend _zgematrix operator+(const  zgsmatrix&, const  zhematrix&);
  inline friend _zgematrix operator+(const  zgsmatrix&, const _zhematrix&);
  inline friend _zgematrix operator+(const  zgsmatrix&, const  zgbmatrix&);
  inline friend _zgematrix operator+(const  zgsmatrix&, const _zgbmatrix&);
  inline friend _zgsmatrix operator+(const  zgsmatrix&, const  zgsmatrix&);
  inline friend _zgsmatrix operator+(const  zgsmatrix&, const _zgsmatrix&);
  inline friend _zgsmatrix operator+(const  zgsmatrix&, const  zhsmatrix&);
  inline friend _zgsmatrix operator+(const  zgsmatrix&, const _zhsmatrix&);
  
  //////// - ////////
  inline friend _zgematrix operator-(const  zgsmatrix&, const  zgematrix&);
  inline friend _zgematrix operator-(const  zgsmatrix&, const _zgematrix&);
  inline friend _zgematrix operator-(const  zgsmatrix&, const  zhematrix&);
  inline friend _zgematrix operator-(const  zgsmatrix&, const _zhematrix&);
  inline friend _zgematrix operator-(const  zgsmatrix&, const  zgbmatrix&);
  inline friend _zgematrix operator-(const  zgsmatrix&, const _zgbmatrix&);
  inline friend _zgsmatrix operator-(const  zgsmatrix&, const  zgsmatrix&);
  inline friend _zgsmatrix operator-(const  zgsmatrix&, const _zgsmatrix&);
  inline friend _zgsmatrix operator-(const  zgsmatrix&, const  zhsmatrix&);
  inline friend _zgsmatrix operator-(const  zgsmatrix&, const _zhsmatrix&);
  
  //////// * ////////
  inline friend _zcovector operator*(const  zgsmatrix&, const  zcovector&);
  inline friend _zcovector operator*(const  zgsmatrix&, const _zcovector&);
  inline friend _zgematrix operator*(const  zgsmatrix&, const  zgematrix&);
  inline friend _zgematrix operator*(const  zgsmatrix&, const _zgematrix&);  
  inline friend _zgematrix operator*(const  zgsmatrix&, const  zhematrix&);
  inline friend _zgematrix operator*(const  zgsmatrix&, const _zhematrix&);
  inline friend _zgematrix operator*(const  zgsmatrix&, const  zgbmatrix&);
  inline friend _zgematrix operator*(const  zgsmatrix&, const _zgbmatrix&);  
  inline friend _zgsmatrix operator*(const  zgsmatrix&, const  zgsmatrix&);
  inline friend _zgsmatrix operator*(const  zgsmatrix&, const _zgsmatrix&);
  inline friend _zgsmatrix operator*(const  zgsmatrix&, const  zhsmatrix&);
  inline friend _zgsmatrix operator*(const  zgsmatrix&, const _zhsmatrix&);
  inline friend _zgsmatrix operator*(const  zgsmatrix&, const     double&);
  inline friend _zgsmatrix operator*(const  zgsmatrix&, const     comple&);

  //////// / ////////
  inline friend _zgsmatrix operator/(const  zgsmatrix&, const     double&);
  inline friend _zgsmatrix operator/(const  zgsmatrix&, const     comple&);
  
  //////// double, comple ////////
  inline friend _zgsmatrix operator*(const     double&, const  zgsmatrix&);
  inline friend _zgsmatrix operator*(const     comple&, const  zgsmatrix&);
};
