//=============================================================================
//! (DO NOT USE) Smart-temporary Real Double-precision Symmetric Sparse Matrix Class
class _dssmatrix
{
public:
  ///////////////////////////////////////////////
  /////////////////// objects ///////////////////
  ///////////////////////////////////////////////
  long const& m; //!< matrix row size
  //mutable long const& m; //!< matrix row size
  mutable long n; //!< matrix column size
  mutable std::vector<dcomponent> data; //!< matrix data
  mutable std::vector< std::vector<uint32_t> > line; //!< vector of vector to store the entry information of component for each row and column
  
  ///////////////////////////////////////////////
  ///////////////// constructors ////////////////
  ///////////////////////////////////////////////
  inline _dssmatrix();
  inline _dssmatrix(const _dssmatrix&);
  inline ~_dssmatrix(); //destructor
  
  ///////////////////////////////////////////////
  ////////////////// functions //////////////////
  ///////////////////////////////////////////////
  //////// cast ////////
  inline _zhsmatrix to_zhsmatrix() const;
  inline _dgematrix to_dgematrix() const;
  inline _dsymatrix to_dsymatrix() const;
  inline _dgsmatrix to_dgsmatrix() const;
  
  //////// io ////////
  inline double operator()(const long&, const long&) const;//not return double&
  inline friend std::ostream& operator<<(std::ostream&, const _dssmatrix&);
  inline void write(const char *) const;

  //////// misc ////////
  inline void nullify() const;
  inline void destroy() const;
  
  //////// calc ////////
  inline friend _dssmatrix t(const dssmatrix&);
  inline friend void idamax(long&, long&, const dssmatrix&);
  inline friend double damax(const dssmatrix&);
  
  ///////////////////////////////////////////////
  ///////////// numerical operators /////////////
  ///////////////////////////////////////////////
  //////// unary ////////
  inline friend const _dssmatrix& operator+(const _dssmatrix&);
  inline friend _dssmatrix operator-(const _dssmatrix&);
  
  //////// + ////////
  inline friend _dgematrix operator+(const _dssmatrix&, const  dgematrix&);
  inline friend _dgematrix operator+(const _dssmatrix&, const _dgematrix&);
  inline friend _dgematrix operator+(const _dssmatrix&, const  dsymatrix&);
  inline friend _dgematrix operator+(const _dssmatrix&, const _dsymatrix&);
  inline friend _dgematrix operator+(const _dssmatrix&, const  dgbmatrix&);
  inline friend _dgematrix operator+(const _dssmatrix&, const _dgbmatrix&);
  inline friend _dgsmatrix operator+(const _dssmatrix&, const  dgsmatrix&);
  inline friend _dgsmatrix operator+(const _dssmatrix&, const _dgsmatrix&);
  inline friend _dssmatrix operator+(const _dssmatrix&, const  dssmatrix&);
  inline friend _dssmatrix operator+(const _dssmatrix&, const _dssmatrix&);
  
  //////// - ////////
  inline friend _dgematrix operator-(const _dssmatrix&, const  dgematrix&);
  inline friend _dgematrix operator-(const _dssmatrix&, const _dgematrix&);
  inline friend _dgematrix operator-(const _dssmatrix&, const  dsymatrix&);
  inline friend _dgematrix operator-(const _dssmatrix&, const _dsymatrix&);
  inline friend _dgematrix operator-(const _dssmatrix&, const  dgbmatrix&);
  inline friend _dgematrix operator-(const _dssmatrix&, const _dgbmatrix&);
  inline friend _dgsmatrix operator-(const _dssmatrix&, const  dgsmatrix&);
  inline friend _dgsmatrix operator-(const _dssmatrix&, const _dgsmatrix&);
  inline friend _dssmatrix operator-(const _dssmatrix&, const  dssmatrix&);
  inline friend _dssmatrix operator-(const _dssmatrix&, const _dssmatrix&);
  
  //////// * ////////
  inline friend _dgematrix operator*(const _dssmatrix&, const  dgematrix&);
  inline friend _dgematrix operator*(const _dssmatrix&, const _dgematrix&);
  inline friend _dgematrix operator*(const _dssmatrix&, const  dsymatrix&);
  inline friend _dgematrix operator*(const _dssmatrix&, const _dsymatrix&);
  inline friend _dgematrix operator*(const _dssmatrix&, const  dgbmatrix&);
  inline friend _dgematrix operator*(const _dssmatrix&, const _dgbmatrix&);
  inline friend _dgematrix operator*(const _dssmatrix&, const  dgsmatrix&);
  inline friend _dgematrix operator*(const _dssmatrix&, const _dgsmatrix&);
  inline friend _dssmatrix operator*(const _dssmatrix&, const  dssmatrix&);
  inline friend _dssmatrix operator*(const _dssmatrix&, const _dssmatrix&);
  inline friend _dssmatrix operator*(const _dssmatrix&, const     double&);
  
  //////// / ////////
  inline friend _dssmatrix operator/(const _dssmatrix&, const     double&);
  
  //////// double ////////
  inline friend _dssmatrix operator*(const     double&, const _dssmatrix&);
};
