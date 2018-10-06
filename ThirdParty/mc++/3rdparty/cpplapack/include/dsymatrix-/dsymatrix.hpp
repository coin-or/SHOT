//=============================================================================
//! Real Double-precision Symmetric Matrix Class [l-type (UPLO=l) Strage]
class dsymatrix
{
public:
  ///////////////////////////////////////////////
  /////////////////// objects ///////////////////
  ///////////////////////////////////////////////
  long const& m; //!< matrix row size
  long n; //!< matrix column size
  double* array; //!< 1D array to store matrix data
  double** darray; //!< array of pointers of column head addresses
  
  ///////////////////////////////////////////////
  ///////////////// constructors ////////////////
  ///////////////////////////////////////////////
  inline dsymatrix();
  inline dsymatrix(const dsymatrix&);
  inline dsymatrix(const _dsymatrix&);
  inline dsymatrix(const long&);
  inline dsymatrix(const char*);
  inline ~dsymatrix(); //destructor
  
  ///////////////////////////////////////////////
  ////////////////// functions //////////////////
  ///////////////////////////////////////////////
  //////// cast ////////
  inline _zhematrix to_zhematrix() const;
  inline _dgematrix to_dgematrix() const;
  inline _dssmatrix to_dssmatrix(const double eps=-1) const;
  
  //////// io ////////
  inline double& operator()(const long&, const long&);
  inline double operator()(const long&, const long&) const;
  inline dsymatrix& set(const long&, const long&, const double&);
  inline friend std::ostream& operator<<(std::ostream&, const dsymatrix&);
  inline void write(const char*) const;
  inline void read(const char*);
  
  //////// misc ////////
  inline void complete() const;
  inline void clear();
  inline dsymatrix& zero();
  inline dsymatrix& identity();
  inline void chsign();
  inline void copy(const dsymatrix&);
  inline void shallow_copy(const _dsymatrix&);
  inline dsymatrix& resize(const long&);
  inline _drovector row(const long&) const;
  inline _dcovector col(const long&) const;
  inline friend void swap(dsymatrix&, dsymatrix&);
  inline friend _dsymatrix _(dsymatrix&);
  
  //////// calc ////////
  inline friend _dsymatrix t(const dsymatrix&);
  inline friend _dsymatrix i(const dsymatrix&);
  inline friend void idamax(long&, long&, const dsymatrix&);
  inline friend double damax(const dsymatrix&);
  
  //////// lapack ////////
  inline long dsysv(dgematrix&);
  inline long dsysv(dcovector&);
  inline long dsyev(std::vector<double>&, const bool&);
  inline long dsyev(std::vector<double>&, std::vector<dcovector>&);
  inline long dsyev(std::vector<double>&, std::vector<drovector>&);
  inline long dsygv(dsymatrix&, std::vector<double>&);
  inline long dsygv(dsymatrix&, std::vector<double>&, std::vector<dcovector>&);
  
  ///////////////////////////////////////////////
  ///////////// numerical operators /////////////
  ///////////////////////////////////////////////
  //////// = ////////
  inline dsymatrix& operator=(const  dsymatrix&);
  inline dsymatrix& operator=(const _dsymatrix&);
  
  //////// += ////////
  inline dsymatrix& operator+=(const  dsymatrix&);
  inline dsymatrix& operator+=(const _dsymatrix&);
  
  //////// -= ////////
  inline dsymatrix& operator-=(const  dsymatrix&);
  inline dsymatrix& operator-=(const _dsymatrix&);
  
  //////// *= ////////
  inline dsymatrix& operator*=(const  dsymatrix&);
  inline dsymatrix& operator*=(const _dsymatrix&);
  inline dsymatrix& operator*=(const     double&);
  
  //////// /= ////////
  inline dsymatrix& operator/=(const     double&);
  
  //////// unary ////////
  inline friend const dsymatrix& operator+(const dsymatrix&);
  inline friend _dsymatrix operator-(const  dsymatrix&);
  
  //////// + ////////
  inline friend _dgematrix operator+(const  dsymatrix&, const  dgematrix&);
  inline friend _dgematrix operator+(const  dsymatrix&, const _dgematrix&);
  inline friend _dsymatrix operator+(const  dsymatrix&, const  dsymatrix&);
  inline friend _dsymatrix operator+(const  dsymatrix&, const _dsymatrix&);
  inline friend _dgematrix operator+(const  dsymatrix&, const  dgbmatrix&);
  inline friend _dgematrix operator+(const  dsymatrix&, const _dgbmatrix&);
  inline friend _dgematrix operator+(const  dsymatrix&, const  dgsmatrix&);
  inline friend _dgematrix operator+(const  dsymatrix&, const _dgsmatrix&);
  inline friend _dsymatrix operator+(const  dsymatrix&, const  dssmatrix&);
  inline friend _dsymatrix operator+(const  dsymatrix&, const _dssmatrix&);
  
  //////// - ////////
  inline friend _dgematrix operator-(const  dsymatrix&, const  dgematrix&);
  inline friend _dgematrix operator-(const  dsymatrix&, const _dgematrix&);
  inline friend _dsymatrix operator-(const  dsymatrix&, const  dsymatrix&);
  inline friend _dsymatrix operator-(const  dsymatrix&, const _dsymatrix&);
  inline friend _dgematrix operator-(const  dsymatrix&, const  dgbmatrix&);
  inline friend _dgematrix operator-(const  dsymatrix&, const _dgbmatrix&);
  inline friend _dgematrix operator-(const  dsymatrix&, const  dgsmatrix&);
  inline friend _dgematrix operator-(const  dsymatrix&, const _dgsmatrix&);
  inline friend _dsymatrix operator-(const  dsymatrix&, const  dssmatrix&);
  inline friend _dsymatrix operator-(const  dsymatrix&, const _dssmatrix&);
  
  //////// * ////////
  inline friend _dcovector operator*(const  dsymatrix&, const  dcovector&);
  inline friend _dcovector operator*(const  dsymatrix&, const _dcovector&);
  inline friend _dgematrix operator*(const  dsymatrix&, const  dgematrix&);
  inline friend _dgematrix operator*(const  dsymatrix&, const _dgematrix&);
  inline friend _dgematrix operator*(const  dsymatrix&, const  dsymatrix&);
  inline friend _dgematrix operator*(const  dsymatrix&, const _dsymatrix&);
  inline friend _dgematrix operator*(const  dsymatrix&, const  dgbmatrix&);
  inline friend _dgematrix operator*(const  dsymatrix&, const _dgbmatrix&);
  inline friend _dgematrix operator*(const  dsymatrix&, const  dgsmatrix&);
  inline friend _dgematrix operator*(const  dsymatrix&, const _dgsmatrix&);
  inline friend _dsymatrix operator*(const  dsymatrix&, const  dssmatrix&);
  inline friend _dsymatrix operator*(const  dsymatrix&, const _dssmatrix&);
  inline friend _dsymatrix operator*(const  dsymatrix&, const     double&);
  
  //////// / ////////
  inline friend _dsymatrix operator/(const  dsymatrix&, const     double&);
  
  //////// double ////////
  inline friend _dsymatrix operator*(const     double&, const  dsymatrix&);
};
