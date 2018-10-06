//=============================================================================
//! Real Double-precision General Dence Matrix Class
class dgematrix
{
public:
  ///////////////////////////////////////////////
  /////////////////// objects ///////////////////
  ///////////////////////////////////////////////
  long m; //!< matrix row size
  long n; //!< matrix column size
  double* array; //!< 1D array to store matrix data
  double** darray; //!< array of pointers of column head addresses
  
  ///////////////////////////////////////////////
  ///////////////// constructors ////////////////
  ///////////////////////////////////////////////
  inline dgematrix();
  inline dgematrix(const dgematrix&);
  inline dgematrix(const _dgematrix&);
  inline dgematrix(const long&, const long&);
  inline dgematrix(const char*);
  inline ~dgematrix(); //destructor
  
  ///////////////////////////////////////////////
  ////////////////// functions //////////////////
  ///////////////////////////////////////////////
  //////// cast ////////
  inline _zgematrix to_zgematrix() const;
  
  //////// io ////////
  inline double& operator()(const long&, const long&);
  inline double operator()(const long&, const long&) const;
  inline dgematrix& set(const long&, const long&, const double&); //const;
  inline friend std::ostream& operator<<(std::ostream&, const dgematrix&);
  inline void write(const char*) const;
  inline void read(const char*);
  
  //////// misc ////////
  inline void clear();
  inline dgematrix& zero();
  inline dgematrix& identity();
  inline void chsign();
  inline void copy(const dgematrix&);
  inline void shallow_copy(const _dgematrix&);
  inline dgematrix& resize(const long&, const long&);
  inline _drovector row(const long&) const;
  inline _dcovector col(const long&) const;
  inline friend void swap(dgematrix&, dgematrix&);
  inline friend _dgematrix _(dgematrix&);
  
  //////// calc ////////
  inline friend _dgematrix t(const dgematrix&);
  inline friend _dgematrix i(const dgematrix&);
  inline friend void idamax(long&, long&, const dgematrix&);
  inline friend double damax(const dgematrix&);
  
  //////// lapack ////////
  inline long dgesv(dgematrix&);
  inline long dgesv(dcovector&);
  inline long dgels(dgematrix&);
  inline long dgels(dcovector&);
  inline long dgels(dgematrix&, drovector&);
  inline long dgels(dcovector&, double&);
  //inline long dgelss(dcovector&);
  inline long dgelss(dcovector&, dcovector&, long&, const double);
  inline long dgelss(dgematrix&, dcovector&, long&, const double);
  inline long dgeev(std::vector<double>&, std::vector<double>&);
  inline long dgeev(zcovector&);
  inline long dgeev(std::vector<double>&, std::vector<double>&, std::vector<dcovector>&, std::vector<dcovector>&);
  inline long dgeev(std::vector<double>&, std::vector<double>&, std::vector<drovector>&, std::vector<drovector>&);
  inline long dggev(dgematrix&, std::vector<double>&, std::vector<double>&);
  inline long dggev(dgematrix&, std::vector<double>&, std::vector<double>&, std::vector<dcovector>&, std::vector<dcovector>&);
  inline long dggev(dgematrix&, std::vector<double>&, std::vector<double>&, std::vector<drovector>&, std::vector<drovector>&);
  inline long dgesvd(dgbmatrix&);
  inline long dgesvd(dcovector&, dgematrix&, dgematrix&);
  inline long dgglse(dgematrix&, dcovector&, dcovector&, dcovector&);
  
  ///////////////////////////////////////////////
  ///////////// numerical operators /////////////
  ///////////////////////////////////////////////
  //////// = ////////
  inline dgematrix& operator=(const  dgematrix&);
  inline dgematrix& operator=(const _dgematrix&);
  
  //////// += ////////
  inline dgematrix& operator+=(const  dgematrix&);
  inline dgematrix& operator+=(const _dgematrix&);
  inline dgematrix& operator+=(const  dsymatrix&);
  inline dgematrix& operator+=(const _dsymatrix&);
  inline dgematrix& operator+=(const  dgbmatrix&);
  inline dgematrix& operator+=(const _dgbmatrix&);
  inline dgematrix& operator+=(const  dgsmatrix&);
  inline dgematrix& operator+=(const _dgsmatrix&);
  inline dgematrix& operator+=(const  dssmatrix&);
  inline dgematrix& operator+=(const _dssmatrix&);
  
  //////// -= ////////
  inline dgematrix& operator-=(const  dgematrix&);
  inline dgematrix& operator-=(const _dgematrix&);
  inline dgematrix& operator-=(const  dsymatrix&);
  inline dgematrix& operator-=(const _dsymatrix&);
  inline dgematrix& operator-=(const  dgbmatrix&);
  inline dgematrix& operator-=(const _dgbmatrix&);
  inline dgematrix& operator-=(const  dgsmatrix&);
  inline dgematrix& operator-=(const _dgsmatrix&);
  inline dgematrix& operator-=(const  dssmatrix&);
  inline dgematrix& operator-=(const _dssmatrix&);
  
  //////// *= ////////
  inline dgematrix& operator*=(const  dgematrix&);
  inline dgematrix& operator*=(const _dgematrix&);
  inline dgematrix& operator*=(const  dsymatrix&);
  inline dgematrix& operator*=(const _dsymatrix&);
  inline dgematrix& operator*=(const  dgbmatrix&);
  inline dgematrix& operator*=(const _dgbmatrix&);
  inline dgematrix& operator*=(const  dgsmatrix&);
  inline dgematrix& operator*=(const _dgsmatrix&);
  inline dgematrix& operator*=(const  dssmatrix&);
  inline dgematrix& operator*=(const _dssmatrix&);
  inline dgematrix& operator*=(const     double&);
  
  //////// /= ////////
  inline dgematrix& operator/=(const     double&);

  //////// unary ////////
  inline friend const dgematrix& operator+(const dgematrix&);
  inline friend _dgematrix operator-(const  dgematrix&);
  
  //////// + ////////
  inline friend _dgematrix operator+(const  dgematrix&, const  dgematrix&);
  inline friend _dgematrix operator+(const  dgematrix&, const _dgematrix&);
  inline friend _dgematrix operator+(const  dgematrix&, const  dsymatrix&);
  inline friend _dgematrix operator+(const  dgematrix&, const _dsymatrix&);
  inline friend _dgematrix operator+(const  dgematrix&, const  dgbmatrix&);
  inline friend _dgematrix operator+(const  dgematrix&, const _dgbmatrix&);
  inline friend _dgematrix operator+(const  dgematrix&, const  dgsmatrix&);
  inline friend _dgematrix operator+(const  dgematrix&, const _dgsmatrix&);
  inline friend _dgematrix operator+(const  dgematrix&, const  dssmatrix&);
  inline friend _dgematrix operator+(const  dgematrix&, const _dssmatrix&);
  
  //////// - ////////
  inline friend _dgematrix operator-(const  dgematrix&, const  dgematrix&);
  inline friend _dgematrix operator-(const  dgematrix&, const _dgematrix&);
  inline friend _dgematrix operator-(const  dgematrix&, const  dsymatrix&);
  inline friend _dgematrix operator-(const  dgematrix&, const _dsymatrix&);
  inline friend _dgematrix operator-(const  dgematrix&, const  dgbmatrix&);
  inline friend _dgematrix operator-(const  dgematrix&, const _dgbmatrix&);
  inline friend _dgematrix operator-(const  dgematrix&, const  dgsmatrix&);
  inline friend _dgematrix operator-(const  dgematrix&, const _dgsmatrix&);
  inline friend _dgematrix operator-(const  dgematrix&, const  dssmatrix&);
  inline friend _dgematrix operator-(const  dgematrix&, const _dssmatrix&);
  
  //////// * ////////
  inline friend _dcovector operator*(const  dgematrix&, const  dcovector&);
  inline friend _dcovector operator*(const  dgematrix&, const _dcovector&);
  inline friend _dgematrix operator*(const  dgematrix&, const  dgematrix&);
  inline friend _dgematrix operator*(const  dgematrix&, const _dgematrix&);
  inline friend _dgematrix operator*(const  dgematrix&, const  dsymatrix&);
  inline friend _dgematrix operator*(const  dgematrix&, const _dsymatrix&);
  inline friend _dgematrix operator*(const  dgematrix&, const  dgbmatrix&);
  inline friend _dgematrix operator*(const  dgematrix&, const _dgbmatrix&);  
  inline friend _dgematrix operator*(const  dgematrix&, const  dgsmatrix&);
  inline friend _dgematrix operator*(const  dgematrix&, const _dgsmatrix&);
  inline friend _dgematrix operator*(const  dgematrix&, const  dssmatrix&);
  inline friend _dgematrix operator*(const  dgematrix&, const _dssmatrix&);
  inline friend _dgematrix operator*(const  dgematrix&, const     double&);
  
  //////// / ////////
  inline friend _dgematrix operator/(const  dgematrix&, const     double&);
  
  //////// % ////////
  inline friend _drovector operator%(const  dgematrix&, const  dgematrix&);
  
  //////// double ////////
  inline friend _dgematrix operator*(const     double&, const  dgematrix&);
};
