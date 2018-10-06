//=============================================================================
//! (DO NOT USE) Smart-temporary Real Double-precision General Sparse Matrix Class
class _dgsmatrix
{
public:
  ///////////////////////////////////////////////
  /////////////////// objects ///////////////////
  ///////////////////////////////////////////////
  mutable long m; //!< matrix row size
  mutable long n; //!< matrix column size
  mutable std::vector<dcomponent> data; //!< matrix data
  mutable std::vector< std::vector<uint32_t> > rows; //!< array of vector to store the entry information of component for each row
  mutable std::vector< std::vector<uint32_t> > cols; //!< array of vector to store the entry information of component for each column
  
  ///////////////////////////////////////////////
  ///////////////// constructors ////////////////
  ///////////////////////////////////////////////
  inline _dgsmatrix();
  inline _dgsmatrix(const _dgsmatrix&);
  inline ~_dgsmatrix(); //destructor
  
  ///////////////////////////////////////////////
  ////////////////// functions //////////////////
  ///////////////////////////////////////////////
  //////// cast ////////
  inline _zgsmatrix to_zgsmatrix() const;
  inline _dgematrix to_dgematrix() const;
  
  //////// io ////////
  inline double operator()(const long&, const long&) const;//not return double&
  inline friend std::ostream& operator<<(std::ostream&, const _dgsmatrix&);
  inline void write(const char*) const;

  //////// misc ////////
  inline void nullify() const;
  inline void destroy() const;
  
  //////// calc ////////
  inline friend _dgsmatrix t(const dgsmatrix&);
  inline friend void idamax(long&, long&, const dgsmatrix&);
  inline friend double damax(const dgsmatrix&);
  
  ///////////////////////////////////////////////
  ///////////// numerical operators /////////////
  ///////////////////////////////////////////////
  //////// unary ////////
  inline friend const _dgsmatrix& operator+(const _dgsmatrix&);
  inline friend _dgsmatrix operator-(const _dgsmatrix&);
  
  //////// + ////////
  inline friend _dgematrix operator+(const _dgsmatrix&, const  dgematrix&);
  inline friend _dgematrix operator+(const _dgsmatrix&, const _dgematrix&);
  inline friend _dgematrix operator+(const _dgsmatrix&, const  dsymatrix&);
  inline friend _dgematrix operator+(const _dgsmatrix&, const _dsymatrix&);
  inline friend _dgematrix operator+(const _dgsmatrix&, const  dgbmatrix&);
  inline friend _dgematrix operator+(const _dgsmatrix&, const _dgbmatrix&);
  inline friend _dgsmatrix operator+(const _dgsmatrix&, const  dgsmatrix&);
  inline friend _dgsmatrix operator+(const _dgsmatrix&, const _dgsmatrix&);
  inline friend _dgsmatrix operator+(const _dgsmatrix&, const  dssmatrix&);
  inline friend _dgsmatrix operator+(const _dgsmatrix&, const _dssmatrix&);
  
  //////// - ////////
  inline friend _dgematrix operator-(const _dgsmatrix&, const  dgematrix&);
  inline friend _dgematrix operator-(const _dgsmatrix&, const _dgematrix&);
  inline friend _dgematrix operator-(const _dgsmatrix&, const  dsymatrix&);
  inline friend _dgematrix operator-(const _dgsmatrix&, const _dsymatrix&);
  inline friend _dgematrix operator-(const _dgsmatrix&, const  dgbmatrix&);
  inline friend _dgematrix operator-(const _dgsmatrix&, const _dgbmatrix&);
  inline friend _dgsmatrix operator-(const _dgsmatrix&, const  dgsmatrix&);
  inline friend _dgsmatrix operator-(const _dgsmatrix&, const _dgsmatrix&);
  inline friend _dgsmatrix operator-(const _dgsmatrix&, const  dssmatrix&);
  inline friend _dgsmatrix operator-(const _dgsmatrix&, const _dssmatrix&);
  
  //////// * ////////
  inline friend _dcovector operator*(const _dgsmatrix&, const  dcovector&);
  inline friend _dcovector operator*(const _dgsmatrix&, const _dcovector&);
  inline friend _dgematrix operator*(const _dgsmatrix&, const  dgematrix&);
  inline friend _dgematrix operator*(const _dgsmatrix&, const _dgematrix&);
  inline friend _dgematrix operator*(const _dgsmatrix&, const  dsymatrix&);
  inline friend _dgematrix operator*(const _dgsmatrix&, const _dsymatrix&);
  inline friend _dgematrix operator*(const _dgsmatrix&, const  dgbmatrix&);
  inline friend _dgematrix operator*(const _dgsmatrix&, const _dgbmatrix&);
  inline friend _dgsmatrix operator*(const _dgsmatrix&, const  dgsmatrix&);
  inline friend _dgsmatrix operator*(const _dgsmatrix&, const _dgsmatrix&);
  inline friend _dgsmatrix operator*(const _dgsmatrix&, const  dssmatrix&);
  inline friend _dgsmatrix operator*(const _dgsmatrix&, const _dssmatrix&);
  inline friend _dgsmatrix operator*(const _dgsmatrix&, const     double&);
  
  //////// / ////////
  inline friend _dgsmatrix operator/(const _dgsmatrix&, const     double&);
  
  //////// double ////////
  inline friend _dgsmatrix operator*(const     double&, const _dgsmatrix&);
};
