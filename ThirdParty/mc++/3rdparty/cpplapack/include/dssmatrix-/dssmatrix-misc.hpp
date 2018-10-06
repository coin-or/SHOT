//=============================================================================
/*! clear all the matrix data and set the sizes 0 */
inline void dssmatrix::clear()
{VERBOSE_REPORT;
  n =0;
  data.clear();
  line.clear();
}

//=============================================================================
/*! change the matrix into a zero matrix */
inline dssmatrix& dssmatrix::zero()
{VERBOSE_REPORT;
  data.resize(0);
  for(long i=0; i<n; i++){ line[i].resize(0); }
  return *this;
}

//=============================================================================
/*! change sign(+/-) of the matrix */
inline void dssmatrix::chsign()
{VERBOSE_REPORT;
  for(std::vector<dcomponent>::iterator it=data.begin(); it!=data.end(); it++){
    it->v =-it->v;
  }
}

//=============================================================================
/*! make a deep copy of the matrix */
inline void dssmatrix::copy(const dssmatrix& mat)
{VERBOSE_REPORT;
  n =mat.n;
  data =mat.data;
  line =mat.line;
}

//=============================================================================
/*! make a shallow copy of the matrix\n
  This function is not designed to be used in project codes. */
inline void dssmatrix::shallow_copy(const _dssmatrix& mat)
{VERBOSE_REPORT;
  data.clear();
  line.clear();
  
  n =mat.n;
  data.swap(mat.data);
  line.swap(mat.line);
  
  mat.nullify();
}

//=============================================================================
/*! resize the matrix */
inline dssmatrix& dssmatrix::resize(const long& _n, const long _c, const long _l)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if( _n<0 || _c<0 || _l<0 ){
    ERROR_REPORT;
    std::cerr << "Matrix sizes, the length of arrays, and line size must be positive integers. " << std::endl
              << "Your input was (" << _n << "," << _c << "," << _l << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  n =_n;
  data.resize(0);
  data.reserve(_c);
  line.resize(n);
  for(long i=0; i<n; i++){
    line[i].resize(0); 
    line[i].reserve(_l);
  }
  
  return *this;
}

//=============================================================================
/*! stretch the matrix size */
inline void dssmatrix::stretch(const long& dn)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if( n+dn<0 ){
    ERROR_REPORT;
    std::cerr << "The new matrix size must be larger than zero." << std::endl
              << "Your input was (" << dn << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  //////// zero ////////
  if(dn==0){ return; }

  //////// non-zero ////////
  n +=dn;
  
  if(dn<0){
    //// delete components over the new size ////
    for(std::vector<dcomponent>::reverse_iterator it=data.rbegin(); it!=data.rend(); it++){
      if( long(it->i)>=n ){ del(data.rend()-it-1); }
    }
    //// shrink line ////
    for(long i=0; i<-dn; i++){
      line.pop_back();
    }
  }
  else{//dn>0
    //// expand line ////
    for(long i=0; i<dn; i++){
      line.push_back( std::vector<uint32_t>(0) );
    }
  }
}

//=============================================================================
/*! check if the component is listed */
inline bool dssmatrix::isListed(const long& i, const long& j) const
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if( i<0 || j<0 || n<=i || n<=j ){
    ERROR_REPORT;
    std::cerr << "The required component is out of the matrix size." << std::endl
              << "Your input was (" << i << "," << j << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  const uint32_t ii(std::max(i,j)), jj(std::min(i,j));
  for(std::vector<uint32_t>::const_iterator p=line[ii].begin(); p!=line[ii].end(); p++){
    if(data[*p].i==ii && data[*p].j==jj){ return 1; }
  }
  
  return 0;
}

//=============================================================================
/*! return the element number of the component */
inline long dssmatrix::number(const long& i, const long& j) const
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if( i<0 || j<0 || n<=i || n<=j ){
    ERROR_REPORT;
    std::cerr << "The required component is out of the matrix size." << std::endl
              << "Your input was (" << i << "," << j << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  const uint32_t ii(std::max(i,j)), jj(std::min(i,j));
  for(std::vector<uint32_t>::const_iterator p=line[ii].begin(); p!=line[ii].end(); p++){
    if(data[*p].i==ii && data[*p].j==jj){ return *p; }
  }
  
  return -1;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! get row of the matrix */
inline _drovector dssmatrix::row(const long& _m) const
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if( _m<0 || _m>m ){
    ERROR_REPORT;
    std::cerr << "Input row number must be between 0 and " << m << "." << std::endl
              << "Your input was " << _m << "." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  drovector vec(n);
  vec.zero();
  for(std::vector<uint32_t>::const_iterator p=line[_m].begin(); p!=line[_m].end(); p++){
    if(long(data[*p].i)==_m){
      vec(data[*p].j) =data[*p].v;
    }
    else{
      vec(data[*p].i) =data[*p].v;
    }
  }
  return _(vec);
}

//=============================================================================
/*! get column of the matrix */
inline _dcovector dssmatrix::col(const long& _n) const
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if( _n<0 || _n>n ){
    ERROR_REPORT;
    std::cerr << "Input row number must be between 0 and " << n << "." << std::endl
              << "Your input was " << _n << "." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  dcovector vec(m);
  vec.zero();
  for(std::vector<uint32_t>::const_iterator p=line[_n].begin(); p!=line[_n].end(); p++){
    if(long(data[*p].i)==_n){
      vec(data[*p].j) =data[*p].v;
    }
    else{
      vec(data[*p].i) =data[*p].v;
    }
  }
  return _(vec);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! erase components less than DBL_MIN */
inline void dssmatrix::diet(const double eps)
{VERBOSE_REPORT;
  for(std::vector<dcomponent>::reverse_iterator it=data.rbegin(); it!=data.rend(); it++){
    if( fabs(it->v)<eps ){ del(data.rend()-it-1); }
  }
}

//=============================================================================
/*! reorder components so that all diagonal componets are placed in front */
inline long dssmatrix::diag_front()
{VERBOSE_REPORT;
  //////// set initial dsize ////////
  long dsize(0);
  for(std::vector<dcomponent>::iterator it=data.begin(); it!=data.end(); it++){
    if(it->i==it->j){ dsize++; }
    else{ break; }
  }
  
  //////// swapping loop ////////
  for(std::vector<dcomponent>::reverse_iterator it=data.rbegin(); it!=data.rend()-dsize; it++){
    if(it->i==it->j){//is diag
      long c(data.rend()-it-1);//current it's index
      long i(data[dsize].i), j(data[dsize].j), k(it->i);
      //// search (k,k) line ////
      for(std::vector<uint32_t>::iterator p=line[k].begin(); p!=line[k].end(); p++){
        if(long(data[*p].i)==k && long(data[*p].j)==k){ *p=dsize; }
      }
      //// search (i,j) line ////
      for(std::vector<uint32_t>::iterator p=line[i].begin(); p!=line[i].end(); p++){
        if(long(*p)==dsize){ *p=c; }
      }
      //// search (j,i) line ////
      if(i!=j){
        for(std::vector<uint32_t>::iterator p=line[j].begin(); p!=line[j].end(); p++){
          if(long(*p)==dsize){ *p=c; }
        }
      }
      else{//i==j
        it--;
      }
      //// swap ////
      std::swap(data[dsize],data[c]);
      //// update ////
      dsize++;
    }
  }
  
  return dsize;
}

//=============================================================================
/*! reorder components */
inline void dssmatrix::reorder(const bool mode)
{VERBOSE_REPORT;
  //// sort data ////
  if(mode==0){
    std::sort(data.begin(), data.end(), dcomponent::ilt);
  }
  else{
    std::sort(data.begin(), data.end(), dcomponent::jlt);
  }
  //// rebuild line ////
  rebuild();
}

//=============================================================================
/*! rebuild line */
inline void dssmatrix::rebuild()
{VERBOSE_REPORT;
  //// clear line ////
  for(long i=0; i<n; i++){ line[i].resize(0); }
  
  //// build line ////
  uint32_t c(0);
  for(std::vector<dcomponent>::iterator it=data.begin(); it!=data.end(); it++){
    line[it->i].push_back(c);
    if( (it->i) != (it->j) ){
      line[it->j].push_back(c);
    }
    c++;
  }
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! health checkup */
inline void dssmatrix::checkup()
{VERBOSE_REPORT;
  //////// write ////////
  for(std::vector<dcomponent>::const_iterator it=data.begin(); it!=data.end(); it++){
    std::cerr << "array[" << it-data.begin() << "] = (" << it->i << "," << it->j << ") = " << it->v << std::endl;
  }
  std::cerr << std::endl;
  
  for(long i=0; i<n; i++){
    std::cerr << "line[" << i << "] =" << std::flush;
    for(unsigned long k=0; k<line[i].size(); k++){
      std::cerr << " " << line[i][k] << std::flush;
    }
    std::cerr << std::endl;
  }
  std::cerr << std::endl;
  
  //////// Elements ////////
  for(std::vector<dcomponent>::const_iterator it=data.begin(); it!=data.end(); it++){
    //// m bound ////
    if(long(it->i)>=n){
      ERROR_REPORT;
      std::cerr << "The indx of the " << it-data.begin() << "th element is out of the matrix size." << std::endl
                << "Its i index was " << it->i << "." << std::endl;
      exit(1);
    }
    
    //// n bound ////
    if(long(it->j)>=n){
      ERROR_REPORT;
      std::cerr << "The jndx of the " << it-data.begin() << "th element is out of the matrix size." << std::endl
                << "Its j index was " << it->j << "." << std::endl;
      exit(1);
    }
    
    //// double-listed ////
    for(std::vector<dcomponent>::const_iterator IT=it+1; IT!=data.end(); IT++){
      if( it->i==IT->i && it->j==IT->j ){
        ERROR_REPORT;
        std::cerr << "The (" << it->i << ", " << it->j << ") component is double-listed at the " << it-data.begin() << "th and the" << IT-data.begin() << "the elements."<< std::endl;
        exit(1);
      }
    }
  }
  
  //////// NOTE ////////
  std::cerr << "# [NOTE]@dssmatrix::checkup(): This symmetric sparse matrix is fine." << std::endl;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! swap two matrices */
inline void swap(dssmatrix& A, dssmatrix& B)
{VERBOSE_REPORT;
  std::swap(A.n,B.n);
  std::swap(A.data,B.data);
  std::swap(A.line,B.line);
}

//=============================================================================
/*! convert user object to smart-temporary object */
inline _dssmatrix _(dssmatrix& mat)
{VERBOSE_REPORT;
  _dssmatrix newmat;
  
  //////// shallow copy ////////
  newmat.n =mat.n;
  std::swap(newmat.data,mat.data);
  std::swap(newmat.line,mat.line);
  
  //////// nullify ////////
  mat.n =0;
  
  return newmat;
}
