//=============================================================================
//! Real Double-precision General Sparse Matrix Class
class dgsmatrix
{
public:
  ///////////////////////////////////////////////
  /////////////////// objects ///////////////////
  ///////////////////////////////////////////////
  long m; //!< matrix row size
  long n; //!< matrix column size
  std::vector<dcomponent> data; //!< matrix data
  std::vector< std::vector<uint32_t> > rows; //!< array of vector to store the entry information of component for each row
  std::vector< std::vector<uint32_t> > cols; //!< array of vector to store the entry information of component for each column
  
  ///////////////////////////////////////////////
  ///////////////// constructors ////////////////
  ///////////////////////////////////////////////
  inline dgsmatrix();
  inline dgsmatrix(const dgsmatrix&);
  inline dgsmatrix(const _dgsmatrix&);
  inline dgsmatrix(const long&, const long&, const long=0);
  inline dgsmatrix(const char*);
  inline ~dgsmatrix(); //destructor
  
  ///////////////////////////////////////////////
  ////////////////// functions //////////////////
  ///////////////////////////////////////////////
  //////// cast ////////
  inline _zgsmatrix to_zgsmatrix() const;
  inline _dgematrix to_dgematrix() const;
  
  //////// io ////////
  inline double operator()(const long&, const long&) const;
  inline double& operator()(const long&, const long&);
  inline dgsmatrix& put(const long&, const long&, const double&);
  inline dgsmatrix& del(const long, const long); //<-- NOT (const long&)
  inline dgsmatrix& del(const long); //<-- NOT (const long&)
  inline friend std::ostream& operator<<(std::ostream&, const dgsmatrix&);
  inline void write(const char*) const;
  inline void read(const char*);

  //////// misc ////////
  inline void clear();
  inline dgsmatrix& zero();
  inline void chsign();
  inline void copy(const dgsmatrix&);
  inline void shallow_copy(const _dgsmatrix&);
  inline dgsmatrix& resize(const long&, const long&, const long=0, const long=0);
  inline void stretch(const long&, const long&);
  inline bool isListed(const long&, const long&) const;
  inline long number(const long&, const long&);
  inline void diet(const double=DBL_MIN);
  inline void checkup();
  inline _drovector row(const long&) const;
  inline _dcovector col(const long&) const;
  inline friend void swap(dgsmatrix&, dgsmatrix&);
  inline friend _dgsmatrix _(dgsmatrix&);
  
  //////// calc ////////
  inline friend _dgsmatrix t(const dgsmatrix&);
  inline friend void idamax(long&, long&, const dgsmatrix&);
  inline friend double damax(const dgsmatrix&);
  
  ///////////////////////////////////////////////
  ///////////// numerical operators /////////////
  ///////////////////////////////////////////////
  //////// = ////////
  inline dgsmatrix& operator=(const  dgsmatrix&);
  inline dgsmatrix& operator=(const _dgsmatrix&);
  
  //////// += ////////
  inline dgsmatrix& operator+=(const  dgsmatrix&);
  inline dgsmatrix& operator+=(const _dgsmatrix&);
  
  //////// -= ////////
  inline dgsmatrix& operator-=(const  dgsmatrix&);
  inline dgsmatrix& operator-=(const _dgsmatrix&);
  
  //////// *= ////////
  inline dgsmatrix& operator*=(const  dgsmatrix&);
  inline dgsmatrix& operator*=(const _dgsmatrix&);
  inline dgsmatrix& operator*=(const     double&);
  
  //////// /= ////////
  inline dgsmatrix& operator/=(const     double&);

  //////// unary ////////
  inline friend const dgsmatrix& operator+(const dgsmatrix&);
  inline friend _dgsmatrix operator-(const  dgsmatrix&);
  
  //////// + ////////
  inline friend _dgematrix operator+(const  dgsmatrix&, const  dgematrix&);
  inline friend _dgematrix operator+(const  dgsmatrix&, const _dgematrix&);
  inline friend _dgematrix operator+(const  dgsmatrix&, const  dsymatrix&);
  inline friend _dgematrix operator+(const  dgsmatrix&, const _dsymatrix&);
  inline friend _dgematrix operator+(const  dgsmatrix&, const  dgbmatrix&);
  inline friend _dgematrix operator+(const  dgsmatrix&, const _dgbmatrix&);
  inline friend _dgsmatrix operator+(const  dgsmatrix&, const  dgsmatrix&);
  inline friend _dgsmatrix operator+(const  dgsmatrix&, const _dgsmatrix&);
  inline friend _dgsmatrix operator+(const  dgsmatrix&, const  dssmatrix&);
  inline friend _dgsmatrix operator+(const  dgsmatrix&, const _dssmatrix&);
  
  //////// - ////////
  inline friend _dgematrix operator-(const  dgsmatrix&, const  dgematrix&);
  inline friend _dgematrix operator-(const  dgsmatrix&, const _dgematrix&);
  inline friend _dgematrix operator-(const  dgsmatrix&, const  dsymatrix&);
  inline friend _dgematrix operator-(const  dgsmatrix&, const _dsymatrix&);
  inline friend _dgematrix operator-(const  dgsmatrix&, const  dgbmatrix&);
  inline friend _dgematrix operator-(const  dgsmatrix&, const _dgbmatrix&);
  inline friend _dgsmatrix operator-(const  dgsmatrix&, const  dgsmatrix&);
  inline friend _dgsmatrix operator-(const  dgsmatrix&, const _dgsmatrix&);
  inline friend _dgsmatrix operator-(const  dgsmatrix&, const  dssmatrix&);
  inline friend _dgsmatrix operator-(const  dgsmatrix&, const _dssmatrix&);
  
  //////// * ////////
  inline friend _dcovector operator*(const  dgsmatrix&, const  dcovector&);
  inline friend _dcovector operator*(const  dgsmatrix&, const _dcovector&);
  inline friend _dgematrix operator*(const  dgsmatrix&, const  dgematrix&);
  inline friend _dgematrix operator*(const  dgsmatrix&, const _dgematrix&);  
  inline friend _dgematrix operator*(const  dgsmatrix&, const  dsymatrix&);
  inline friend _dgematrix operator*(const  dgsmatrix&, const _dsymatrix&);
  inline friend _dgematrix operator*(const  dgsmatrix&, const  dgbmatrix&);
  inline friend _dgematrix operator*(const  dgsmatrix&, const _dgbmatrix&);  
  inline friend _dgsmatrix operator*(const  dgsmatrix&, const  dgsmatrix&);
  inline friend _dgsmatrix operator*(const  dgsmatrix&, const _dgsmatrix&);
  inline friend _dgsmatrix operator*(const  dgsmatrix&, const  dssmatrix&);
  inline friend _dgsmatrix operator*(const  dgsmatrix&, const _dssmatrix&);
  inline friend _dgsmatrix operator*(const  dgsmatrix&, const     double&);
  
  //////// / ////////
  inline friend _dgsmatrix operator/(const  dgsmatrix&, const     double&);

  //////// double ////////
  inline friend _dgsmatrix operator*(const     double&, const  dgsmatrix&);
};
