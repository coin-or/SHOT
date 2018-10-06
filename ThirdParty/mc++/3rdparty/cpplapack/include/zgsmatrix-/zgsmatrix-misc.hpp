//=============================================================================
/*! clear all the matrix data and set the sizes 0 */
inline void zgsmatrix::clear()
{VERBOSE_REPORT;
  m =0;
  n =0;
  data.clear();
  rows.clear();
  cols.clear();
}

//=============================================================================
/*! change the matrix into a zero matrix */
inline zgsmatrix& zgsmatrix::zero()
{VERBOSE_REPORT;
  data.resize(0);
  for(long i=0; i<m; i++){ rows[i].resize(0); }
  for(long j=0; j<n; j++){ cols[j].resize(0); }
  return *this;
}

//=============================================================================
/*! change sign(+/-) of the matrix */
inline void zgsmatrix::chsign()
{VERBOSE_REPORT;
  for(std::vector<zcomponent>::iterator it=data.begin(); it!=data.end(); it++){
    it->v =-it->v;
  }
}

//=============================================================================
/*! make a deep copy of the matrix */
inline void zgsmatrix::copy(const zgsmatrix& mat)
{VERBOSE_REPORT;
  m =mat.m;
  n =mat.n;
  data =mat.data;
  rows =mat.rows;
  cols =mat.cols;
}

//=============================================================================
/*! make a shallow copy of the matrix\n
  This function is not designed to be used in project codes. */
inline void zgsmatrix::shallow_copy(const _zgsmatrix& mat)
{VERBOSE_REPORT;
  data.clear();
  rows.clear();
  cols.clear();

  m =mat.m;
  n =mat.n;
  data.swap(mat.data);
  rows.swap(mat.rows);
  cols.swap(mat.cols);

  mat.nullify();
}

//=============================================================================
/*! resize the matrix */
inline zgsmatrix& zgsmatrix::resize(const long& _m, const long& _n, const long _c, const long _l)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if( _m<0 || _n<0 || _c<0 ){
    ERROR_REPORT;
    std::cerr << "Matrix sizes and the length of arrays must be positive integers. " << std::endl
              << "Your input was (" << _m << "," << _n << "," << _c << "," << _l << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  m =_m;
  n =_n;
  data.resize(0);
  data.reserve(_c);
  rows.resize(m);
  for(long i=0; i<m; i++){
    rows[i].resize(0);
    rows[i].reserve(_l);
  }
  cols.resize(n);
  for(long i=0; i<n; i++){
    cols[i].resize(0);
    cols[i].reserve(_l);
  }
  
  return *this;
}

//=============================================================================
/*! stretch the matrix size */
inline void zgsmatrix::stretch(const long& dm, const long& dn)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if( m+dm<0 || n+dn<0 ){
    ERROR_REPORT;
    std::cerr << "The new matrix size must be larger than zero. " << std::endl
              << "Your input was (" << dm << ", " << dn << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  //////// zero ////////
  if(dm==0 && dn==0){ return; }
  
  //////// non-zero ////////
  m +=dm;
  n +=dn;
  
  //// for rows ////
  if(dm<0){
    //// delete components over the new size ////
    for(std::vector<zcomponent>::reverse_iterator it=data.rbegin(); it!=data.rend(); it++){
      if( long(it->i)>=m ){ del(data.rend()-it-1); }
    }
    //// shrink rows ////
    for(long i=0; i<-dm; i++){
      rows.pop_back();
    }
  }
  else{//dm>0
    //// expand rows ////
    for(long i=0; i<dm; i++){
      rows.push_back( std::vector<uint32_t>(0) );
    }
  }

  //// for cols ////
  if(dn<0){
    //// delete components over the new size ////
    for(std::vector<zcomponent>::reverse_iterator it=data.rbegin(); it!=data.rend(); it++){
      if( long(it->j)>=n ){ del(data.rend()-it-1); }
    }
    for(long j=0; j<-dn; j++){
      cols.pop_back();
    }
  }
  else{//dn>0
    //// expand cols ////
    for(long j=0; j<dn; j++){
      cols.push_back( std::vector<uint32_t>(0) );
    }
  }
}

//=============================================================================
/*! check if the component is listed */
inline bool zgsmatrix::isListed(const long& i, const long& j)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if( i<0 || j<0 || m<=i || n<=j ){
    ERROR_REPORT;
    std::cerr << "The required component is out of the matrix size." << std::endl
              << "Your input was (" << i << "," << j << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  for(std::vector<uint32_t>::const_iterator p=rows[i].begin(); p!=rows[i].end(); p++){
    if( long(data[*p].j)==j ){ return 1; }
  }
  
  return 0;
}

//=============================================================================
/*! return the element number of the component */
inline long zgsmatrix::number(const long& i, const long& j)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if( i<0 || j<0 || m<=i || n<=j ){
    ERROR_REPORT;
    std::cerr << "The required component is out of the matrix size." << std::endl
              << "Your input was (" << i << "," << j << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  for(std::vector<uint32_t>::iterator p=rows[i].begin(); p!=rows[i].end(); p++){
    if( long(data[*p].j)==j ){ return *p; }
  }
  
  return -1;
}

//=============================================================================
/*! erase components less than DBL_MIN */
inline void zgsmatrix::diet(const double eps)
{VERBOSE_REPORT;
  for(std::vector<zcomponent>::reverse_iterator it=data.rbegin(); it!=data.rend(); it++){
    if( fabs(it->v.real())<eps && fabs(it->v.imag())<eps ){ del(data.rend()-it-1); }
  }
}

//=============================================================================
/*! health checkup */
inline void zgsmatrix::checkup()
{VERBOSE_REPORT;
  //////////////// data ////////////////
  //////// check i,j ////////
  for(std::vector<zcomponent>::const_iterator it=data.begin(); it!=data.end(); it++){
    if( long(it->i)>=m || long(it->j)>=n ){
      ERROR_REPORT;
      std::cerr << "A component, (" << it->i << ", " << it->j << "), is out of matrix size." << std::endl;
      exit(1);
    }
  }
  //////// check double listing ////////
  
  //////////////// rows ////////////////
  //////////////// cols ////////////////
  
  //////////////// NOTE ////////////////
  std::cerr << "# [NOTE] zgsmatrix::checkup(): This sparse matrix is fine." << std::endl;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! get row of the matrix */
inline _zrovector zgsmatrix::row(const long& _m) const
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if( _m<0 || _m>m ){
    ERROR_REPORT;
    std::cerr << "Input row number must be between 0 and " << m << "." << std::endl
              << "Your input was " << _m << "." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  zrovector vec( zrovector(n).zero() );
  for(std::vector<uint32_t>::const_iterator p=rows[_m].begin(); p!=rows[_m].end(); p++){
    vec(data[*p].j) =data[*p].v;
  }
  return _(vec);
}

//=============================================================================
/*! get column of the matrix */
inline _zcovector zgsmatrix::col(const long& _n) const
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if( _n<0 || _n>n ){
    ERROR_REPORT;
    std::cerr << "Input row number must be between 0 and " << n << "." << std::endl
              << "Your input was " << _n << "." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  zcovector vec( zcovector(m).zero() );
  for(std::vector<uint32_t>::const_iterator p=cols[_n].begin(); p!=cols[_n].end(); p++){
    vec(data[*p].i) =data[*p].v;
  }
  return _(vec);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! swap two matrices */
inline void swap(zgsmatrix& A, zgsmatrix& B)
{VERBOSE_REPORT;
  std::swap(A.n,B.n);
  std::swap(A.m,B.m);
  std::swap(A.data,B.data);
  std::swap(A.rows,B.rows);
  std::swap(A.cols,B.cols);
}

//=============================================================================
/*! convert user object to smart-temporary object */
inline _zgsmatrix _(zgsmatrix& mat)
{VERBOSE_REPORT;
  _zgsmatrix newmat;
  
  //////// shallow copy ////////
  newmat.n =mat.n;
  newmat.m =mat.m;
  std::swap(newmat.data,mat.data);
  std::swap(newmat.rows,mat.rows);
  std::swap(newmat.cols,mat.cols);

  //////// nullify ////////
  mat.m =0;
  mat.n =0;
  
  return newmat;
}
