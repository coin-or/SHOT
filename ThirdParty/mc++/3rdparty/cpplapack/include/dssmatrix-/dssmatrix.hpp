//=============================================================================
//! Real Double-precision Symmetric Sparse Matrix Class
class dssmatrix
{
public:
  ///////////////////////////////////////////////
  /////////////////// objects ///////////////////
  ///////////////////////////////////////////////
  long const& m; //!< matrix row size
  long n; //!< matrix column size
  std::vector<dcomponent> data; //!< matrix data
  std::vector< std::vector<uint32_t> > line; //!< vector of vector to store the entry information of component for each row and column
  
  ///////////////////////////////////////////////
  ///////////////// constructors ////////////////
  ///////////////////////////////////////////////
  inline dssmatrix();
  inline dssmatrix(const dssmatrix&);
  inline dssmatrix(const _dssmatrix&);
  inline dssmatrix(const long&, const long=0);
  inline dssmatrix(const char*);
  inline ~dssmatrix(); //destructor
  
  ///////////////////////////////////////////////
  ////////////////// functions //////////////////
  ///////////////////////////////////////////////
  //////// cast ////////
  inline _zhsmatrix to_zhsmatrix() const;
  inline _dgematrix to_dgematrix() const;
  inline _dsymatrix to_dsymatrix() const;
  inline _dgsmatrix to_dgsmatrix() const;
  
  //////// io ////////
  inline double operator()(const long&, const long&) const;
  inline double& operator()(const long&, const long&);
  inline dssmatrix& put(const long&, const long&, const double&);
  inline dssmatrix& del(const long, const long); //<-- NOT (const long&)
  inline dssmatrix& del(const long); //<-- NOT (const long&)
  inline friend std::ostream& operator<<(std::ostream&, const dssmatrix&);
  inline void write(const char*) const;
  inline void read(const char*);

  //////// misc ////////
  inline void clear();
  inline dssmatrix& zero();
  inline void chsign();
  inline void copy(const dssmatrix&);
  inline void shallow_copy(const _dssmatrix&);
  inline dssmatrix& resize(const long&, const long=0, const long=0);
  inline void stretch(const long&);
  inline bool isListed(const long&, const long&) const;
  inline long number(const long&, const long&) const;
  inline _drovector row(const long&) const;
  inline _dcovector col(const long&) const;
  inline void diet(const double=DBL_MIN);
  inline long diag_front();
  inline void reorder(const bool=0);
  inline void rebuild();
  inline void checkup();
  inline friend void swap(dssmatrix&, dssmatrix&);
  inline friend _dssmatrix _(dssmatrix&);
  
  //////// calc ////////
  inline friend _dssmatrix t(const dssmatrix&);
  inline friend void idamax(long&, long&, const dssmatrix&);
  inline friend double damax(const dssmatrix&);
  
  ///////////////////////////////////////////////
  ///////////// numerical operators /////////////
  ///////////////////////////////////////////////
  //////// = ////////
  inline dssmatrix& operator=(const  dssmatrix&);
  inline dssmatrix& operator=(const _dssmatrix&);
  
  //////// += ////////
  inline dssmatrix& operator+=(const  dssmatrix&);
  inline dssmatrix& operator+=(const _dssmatrix&);
  
  //////// -= ////////
  inline dssmatrix& operator-=(const  dssmatrix&);
  inline dssmatrix& operator-=(const _dssmatrix&);
  
  //////// *= ////////
  inline dssmatrix& operator*=(const     double&);
  
  //////// /= ////////
  inline dssmatrix& operator/=(const     double&);
  
  //////// unary ////////
  inline friend const dssmatrix& operator+(const dssmatrix&);
  inline friend _dssmatrix operator-(const  dssmatrix&);
  
  //////// + ////////
  inline friend _dgematrix operator+(const  dssmatrix&, const  dgematrix&);
  inline friend _dgematrix operator+(const  dssmatrix&, const _dgematrix&);
  inline friend _dgematrix operator+(const  dssmatrix&, const  dsymatrix&);
  inline friend _dgematrix operator+(const  dssmatrix&, const _dsymatrix&);
  inline friend _dgematrix operator+(const  dssmatrix&, const  dgbmatrix&);
  inline friend _dgematrix operator+(const  dssmatrix&, const _dgbmatrix&);
  inline friend _dgsmatrix operator+(const  dssmatrix&, const  dgsmatrix&);
  inline friend _dgsmatrix operator+(const  dssmatrix&, const _dgsmatrix&);
  inline friend _dssmatrix operator+(const  dssmatrix&, const  dssmatrix&);
  inline friend _dssmatrix operator+(const  dssmatrix&, const _dssmatrix&);
  
  //////// - ////////
  inline friend _dgematrix operator-(const  dssmatrix&, const  dgematrix&);
  inline friend _dgematrix operator-(const  dssmatrix&, const _dgematrix&);
  inline friend _dgematrix operator-(const  dssmatrix&, const  dsymatrix&);
  inline friend _dgematrix operator-(const  dssmatrix&, const _dsymatrix&);
  inline friend _dgematrix operator-(const  dssmatrix&, const  dgbmatrix&);
  inline friend _dgematrix operator-(const  dssmatrix&, const _dgbmatrix&);
  inline friend _dgsmatrix operator-(const  dssmatrix&, const  dgsmatrix&);
  inline friend _dgsmatrix operator-(const  dssmatrix&, const _dgsmatrix&);
  inline friend _dssmatrix operator-(const  dssmatrix&, const  dssmatrix&);
  inline friend _dssmatrix operator-(const  dssmatrix&, const _dssmatrix&);
  
  //////// * ////////
  inline friend _dcovector operator*(const  dssmatrix&, const  dcovector&);
  inline friend _dcovector operator*(const  dssmatrix&, const _dcovector&);
  inline friend _dgematrix operator*(const  dssmatrix&, const  dgematrix&);
  inline friend _dgematrix operator*(const  dssmatrix&, const _dgematrix&);  
  inline friend _dgematrix operator*(const  dssmatrix&, const  dsymatrix&);
  inline friend _dgematrix operator*(const  dssmatrix&, const _dsymatrix&);
  inline friend _dgematrix operator*(const  dssmatrix&, const  dgbmatrix&);
  inline friend _dgematrix operator*(const  dssmatrix&, const _dgbmatrix&);  
  inline friend _dgsmatrix operator*(const  dssmatrix&, const  dgsmatrix&);
  inline friend _dgsmatrix operator*(const  dssmatrix&, const _dgsmatrix&);  
  inline friend _dgsmatrix operator*(const  dssmatrix&, const  dssmatrix&);
  inline friend _dgsmatrix operator*(const  dssmatrix&, const _dssmatrix&);
  inline friend _dssmatrix operator*(const  dssmatrix&, const     double&);
  
  //////// / ////////
  inline friend _dssmatrix operator/(const  dssmatrix&, const     double&);
  
  //////// double ////////
  inline friend _dssmatrix operator*(const     double&, const  dssmatrix&);
};
