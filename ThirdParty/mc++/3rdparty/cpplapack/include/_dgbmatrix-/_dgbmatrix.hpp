//=============================================================================
//! (DO NOT USE) Smart-temporary Real Double-precision General Band Matrix Class
class _dgbmatrix
{
public:
  ///////////////////////////////////////////////
  /////////////////// objects ///////////////////
  ///////////////////////////////////////////////
  mutable long m; //!< matrix row size
  mutable long n; //!< matrix column size
  mutable long kl; //!< lower band width 
  mutable long ku; //!< upper band width 
  mutable double* array; //!< 1D array to store matrix data
  mutable double** darray; //!< array of pointers of column head addresses
  
  ///////////////////////////////////////////////
  ///////////////// constructors ////////////////
  ///////////////////////////////////////////////
  inline _dgbmatrix();
  inline _dgbmatrix(const _dgbmatrix&);
  inline ~_dgbmatrix(); //destructor
  
  ///////////////////////////////////////////////
  ////////////////// functions //////////////////
  ///////////////////////////////////////////////
  //////// cast ////////
  inline _zgbmatrix to_zgbmatrix() const;
  inline _dgematrix to_dgematrix() const;
  
  //////// io ////////
  inline double& operator()(const long&, const long&) const;
  inline friend std::ostream& operator<<(std::ostream&, const _dgbmatrix&);
  inline void write(const char*) const;
  
  //////// misc ////////
  inline void nullify() const;
  inline void destroy() const;
  
  //////// calc ////////
  inline friend _dgbmatrix t(const _dgbmatrix&);
  inline friend _dgematrix i(const _dgbmatrix&);
  
  ///////////////////////////////////////////////
  ///////////// numerical operators /////////////
  ///////////////////////////////////////////////
  //////// unary ////////
  inline friend const _dgbmatrix& operator+(const _dgbmatrix&);
  inline friend _dgbmatrix operator-(const _dgbmatrix&);
  
  //////// + ////////
  inline friend _dgematrix operator+(const _dgbmatrix&, const  dgematrix&);
  inline friend _dgematrix operator+(const _dgbmatrix&, const _dgematrix&);
  inline friend _dgematrix operator+(const _dgbmatrix&, const  dsymatrix&);
  inline friend _dgematrix operator+(const _dgbmatrix&, const _dsymatrix&);
  inline friend _dgbmatrix operator+(const _dgbmatrix&, const  dgbmatrix&);
  inline friend _dgbmatrix operator+(const _dgbmatrix&, const _dgbmatrix&);
  inline friend _dgematrix operator+(const _dgbmatrix&, const  dgsmatrix&);
  inline friend _dgematrix operator+(const _dgbmatrix&, const _dgsmatrix&);
  inline friend _dgematrix operator+(const _dgbmatrix&, const  dssmatrix&);
  inline friend _dgematrix operator+(const _dgbmatrix&, const _dssmatrix&);
  
  //////// - ////////
  inline friend _dgematrix operator-(const _dgbmatrix&, const  dgematrix&);
  inline friend _dgematrix operator-(const _dgbmatrix&, const _dgematrix&);
  inline friend _dgematrix operator-(const _dgbmatrix&, const  dsymatrix&);
  inline friend _dgematrix operator-(const _dgbmatrix&, const _dsymatrix&);
  inline friend _dgbmatrix operator-(const _dgbmatrix&, const  dgbmatrix&);
  inline friend _dgbmatrix operator-(const _dgbmatrix&, const _dgbmatrix&);
  inline friend _dgematrix operator-(const _dgbmatrix&, const  dgsmatrix&);
  inline friend _dgematrix operator-(const _dgbmatrix&, const _dgsmatrix&);
  inline friend _dgematrix operator-(const _dgbmatrix&, const  dssmatrix&);
  inline friend _dgematrix operator-(const _dgbmatrix&, const _dssmatrix&);
  
  //////// * ////////
  inline friend _dcovector operator*(const _dgbmatrix&, const  dcovector&);
  inline friend _dcovector operator*(const _dgbmatrix&, const _dcovector&);
  inline friend _dgematrix operator*(const _dgbmatrix&, const  dgematrix&);
  inline friend _dgematrix operator*(const _dgbmatrix&, const _dgematrix&);
  inline friend _dgematrix operator*(const _dgbmatrix&, const  dsymatrix&);
  inline friend _dgematrix operator*(const _dgbmatrix&, const _dsymatrix&);
  inline friend _dgbmatrix operator*(const _dgbmatrix&, const  dgbmatrix&);
  inline friend _dgbmatrix operator*(const _dgbmatrix&, const _dgbmatrix&);
  inline friend _dgematrix operator*(const _dgbmatrix&, const  dgsmatrix&);
  inline friend _dgematrix operator*(const _dgbmatrix&, const _dgsmatrix&);
  inline friend _dgematrix operator*(const _dgbmatrix&, const  dssmatrix&);
  inline friend _dgematrix operator*(const _dgbmatrix&, const _dssmatrix&);
  inline friend _dgbmatrix operator*(const _dgbmatrix&, const     double&);
  
  //////// / ////////
  inline friend _dgbmatrix operator/(const _dgbmatrix&, const     double&);
  
  //////// double ////////
  inline friend _dgbmatrix operator*(const     double&, const _dgbmatrix&);
};
