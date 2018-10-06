//=============================================================================
//! (DO NOT USE) Smart-temporary Real Double-precision General Dence Matrix Class
class _dgematrix
{
public:
  ///////////////////////////////////////////////
  /////////////////// objects ///////////////////
  ///////////////////////////////////////////////
  mutable long m; //!< matrix row size
  mutable long n; //!< matrix column size
  mutable double* array; //!< 1D array to store matrix data
  mutable double** darray; //!< array of pointers of column head addresses

  ///////////////////////////////////////////////
  ///////////////// constructors ////////////////
  ///////////////////////////////////////////////
  inline _dgematrix();
  inline _dgematrix(const _dgematrix&);
  inline ~_dgematrix(); //destructor
  
  ///////////////////////////////////////////////
  ////////////////// functions //////////////////
  ///////////////////////////////////////////////
  //////// cast ////////
  inline _zgematrix to_zgematrix() const;
  
  //////// io ////////
  inline double& operator()(const long&, const long&) const;
  inline friend std::ostream& operator<<(std::ostream&, const dgematrix&);
  inline void write(const char*) const;
  
  //////// misc ////////
  inline void nullify() const;
  inline void destroy() const;
  
  //////// calc ////////
  inline friend _dgematrix t(const _dgematrix&);
  inline friend _dgematrix i(const _dgematrix&);
  inline friend void idamax(long&, long&, const _dgematrix&);
  inline friend double damax(const _dgematrix&);
  
  ///////////////////////////////////////////////
  ///////////// numerical operators /////////////
  ///////////////////////////////////////////////
  //////// unary ////////
  inline friend const _dgematrix& operator+(const _dgematrix&);
  inline friend _dgematrix operator-(const _dgematrix&);
  
  //////// + ////////
  inline friend _dgematrix operator+(const _dgematrix&, const  dgematrix&);
  inline friend _dgematrix operator+(const _dgematrix&, const _dgematrix&);
  inline friend _dgematrix operator+(const _dgematrix&, const  dsymatrix&);
  inline friend _dgematrix operator+(const _dgematrix&, const _dsymatrix&);
  inline friend _dgematrix operator+(const _dgematrix&, const  dgbmatrix&);
  inline friend _dgematrix operator+(const _dgematrix&, const _dgbmatrix&);
  inline friend _dgematrix operator+(const _dgematrix&, const  dgsmatrix&);
  inline friend _dgematrix operator+(const _dgematrix&, const _dgsmatrix&);
  inline friend _dgematrix operator+(const _dgematrix&, const  dssmatrix&);
  inline friend _dgematrix operator+(const _dgematrix&, const _dssmatrix&);
  
  //////// - ////////
  inline friend _dgematrix operator-(const _dgematrix&, const  dgematrix&);
  inline friend _dgematrix operator-(const _dgematrix&, const _dgematrix&);
  inline friend _dgematrix operator-(const _dgematrix&, const  dsymatrix&);
  inline friend _dgematrix operator-(const _dgematrix&, const _dsymatrix&);
  inline friend _dgematrix operator-(const _dgematrix&, const  dgbmatrix&);
  inline friend _dgematrix operator-(const _dgematrix&, const _dgbmatrix&);
  inline friend _dgematrix operator-(const _dgematrix&, const  dgsmatrix&);
  inline friend _dgematrix operator-(const _dgematrix&, const _dgsmatrix&);
  inline friend _dgematrix operator-(const _dgematrix&, const  dssmatrix&);
  inline friend _dgematrix operator-(const _dgematrix&, const _dssmatrix&);
  
  //////// * ////////
  inline friend _dcovector operator*(const _dgematrix&, const  dcovector&);
  inline friend _dcovector operator*(const _dgematrix&, const _dcovector&);
  inline friend _dgematrix operator*(const _dgematrix&, const  dgematrix&);
  inline friend _dgematrix operator*(const _dgematrix&, const _dgematrix&);
  inline friend _dgematrix operator*(const _dgematrix&, const  dsymatrix&);
  inline friend _dgematrix operator*(const _dgematrix&, const _dsymatrix&);
  inline friend _dgematrix operator*(const _dgematrix&, const  dgbmatrix&);
  inline friend _dgematrix operator*(const _dgematrix&, const _dgbmatrix&);
  inline friend _dgematrix operator*(const _dgematrix&, const  dgsmatrix&);
  inline friend _dgematrix operator*(const _dgematrix&, const _dgsmatrix&);
  inline friend _dgematrix operator*(const _dgematrix&, const  dssmatrix&);
  inline friend _dgematrix operator*(const _dgematrix&, const _dssmatrix&);
  inline friend _dgematrix operator*(const _dgematrix&, const     double&);
  
  //////// / ////////
  inline friend _dgematrix operator/(const _dgematrix&, const     double&);
  
  //////// double ////////
  inline friend _dgematrix operator*(const     double&, const _dgematrix&);
};
