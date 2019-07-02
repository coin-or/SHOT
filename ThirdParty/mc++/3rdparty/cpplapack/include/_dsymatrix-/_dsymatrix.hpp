//=============================================================================
//! (DO NOT USE) Smart-temporary Real Double-precision Symmetric Matrix Class
class _dsymatrix
{
public:
  ///////////////////////////////////////////////
  /////////////////// objects ///////////////////
  ///////////////////////////////////////////////
  long const& m; //!< matrix row size
  //mutable long const& m; //!< matrix row size
  mutable long n; //!< matrix column size
  mutable double* array; //!< 1D array to store matrix data
  mutable double** darray; //!< array of pointers of column head addresses
  
  ///////////////////////////////////////////////
  ///////////////// constructors ////////////////
  ///////////////////////////////////////////////
  inline _dsymatrix();
  inline _dsymatrix(const _dsymatrix&);
  inline ~_dsymatrix(); //destructor
  
  ///////////////////////////////////////////////
  ////////////////// functions //////////////////
  ///////////////////////////////////////////////
  //////// cast ////////
  inline _zhematrix to_zhematrix() const;
  inline _dgematrix to_dgematrix() const;
  inline _dssmatrix to_dssmatrix(const double=DBL_MIN) const;
  
  //////// io ////////
  inline double& operator()(const long&, const long&) const;
  inline friend std::ostream& operator<<(std::ostream&, const dsymatrix&);
  inline void write(const char*) const;
  
  //////// misc ////////
  inline void nullify() const;
  inline void destroy() const;
  inline void complete() const;
  
  //////// calc ////////
  inline friend _dsymatrix t(const _dsymatrix&);
  inline friend _dsymatrix i(const _dsymatrix&);
  inline friend void idamax(long&, long&, const _dsymatrix&);
  inline friend double damax(const _dsymatrix&);
  
  ///////////////////////////////////////////////
  ///////////// numerical operators /////////////
  ///////////////////////////////////////////////
  //////// unary ////////
  inline friend const _dsymatrix& operator+(const _dsymatrix&);
  inline friend _dsymatrix operator-(const _dsymatrix&);
  
  //////// + ////////
  inline friend _dgematrix operator+(const _dsymatrix&, const  dgematrix&);
  inline friend _dgematrix operator+(const _dsymatrix&, const _dgematrix&);
  inline friend _dsymatrix operator+(const _dsymatrix&, const  dsymatrix&);
  inline friend _dsymatrix operator+(const _dsymatrix&, const _dsymatrix&);
  inline friend _dgematrix operator+(const _dsymatrix&, const  dgbmatrix&);
  inline friend _dgematrix operator+(const _dsymatrix&, const _dgbmatrix&);
  inline friend _dgematrix operator+(const _dsymatrix&, const  dgsmatrix&);
  inline friend _dgematrix operator+(const _dsymatrix&, const _dgsmatrix&);
  inline friend _dsymatrix operator+(const _dsymatrix&, const  dssmatrix&);
  inline friend _dsymatrix operator+(const _dsymatrix&, const _dssmatrix&);
  
  //////// - ////////
  inline friend _dgematrix operator-(const _dsymatrix&, const  dgematrix&);  
  inline friend _dgematrix operator-(const _dsymatrix&, const _dgematrix&);
  inline friend _dsymatrix operator-(const _dsymatrix&, const  dsymatrix&);
  inline friend _dsymatrix operator-(const _dsymatrix&, const _dsymatrix&);
  inline friend _dgematrix operator-(const _dsymatrix&, const  dgbmatrix&);  
  inline friend _dgematrix operator-(const _dsymatrix&, const _dgbmatrix&);
  inline friend _dgematrix operator-(const _dsymatrix&, const  dgsmatrix&);
  inline friend _dgematrix operator-(const _dsymatrix&, const _dgsmatrix&);
  inline friend _dsymatrix operator-(const _dsymatrix&, const  dssmatrix&);
  inline friend _dsymatrix operator-(const _dsymatrix&, const _dssmatrix&);
  
  //////// * ////////
  inline friend _dcovector operator*(const _dsymatrix&, const  dcovector&);
  inline friend _dcovector operator*(const _dsymatrix&, const _dcovector&);
  inline friend _dgematrix operator*(const _dsymatrix&, const  dgematrix&);
  inline friend _dgematrix operator*(const _dsymatrix&, const _dgematrix&);
  inline friend _dgematrix operator*(const _dsymatrix&, const  dsymatrix&);
  inline friend _dgematrix operator*(const _dsymatrix&, const _dsymatrix&);
  inline friend _dgematrix operator*(const _dsymatrix&, const  dgbmatrix&);
  inline friend _dgematrix operator*(const _dsymatrix&, const _dgbmatrix&);
  inline friend _dgematrix operator*(const _dsymatrix&, const  dgsmatrix&);
  inline friend _dgematrix operator*(const _dsymatrix&, const _dgsmatrix&);
  inline friend _dgematrix operator*(const _dsymatrix&, const  dssmatrix&);
  inline friend _dgematrix operator*(const _dsymatrix&, const _dssmatrix&);
  inline friend _dsymatrix operator*(const _dsymatrix&, const     double&);
  
  //////// / ////////
  inline friend _dsymatrix operator/(const _dsymatrix&, const     double&);
  
  //////// double ////////
  inline friend _dsymatrix operator*(const     double&, const _dsymatrix&);
};
