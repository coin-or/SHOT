//=============================================================================
/*! operator() for non-const object */
inline zhecomplex zhematrix::operator()(const long& i, const long& j)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if( i<0 || j<0 || n<=i || n<=j ){
    ERROR_REPORT;
    std::cerr << "The required component is out of the matrix size." << std::endl
              << "Your input was (" << i << "," << j << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  if(i>=j){ return zhecomplex(i,j, darray[j][i]); }
  else    { return zhecomplex(i,j, darray[i][j]); }
}

//=============================================================================
/*! operator() for const object */
inline comple zhematrix::operator()(const long& i, const long& j) const
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if( i<0 || j<0 || n<=i || n<=j ){
    ERROR_REPORT;
    std::cerr << "The required component is out of the matrix size." << std::endl
              << "Your input was (" << i << "," << j << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  if(i>=j){ return darray[j][i]; }
  else    { return std::conj(darray[i][j]); }
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! set value for const object */
inline zhematrix& zhematrix::set(const long& i, const long& j, const comple& v)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if( i<0 || j<0 || n<=i || n<=j ){
    ERROR_REPORT;
    std::cerr << "The required component is out of the matrix size." << std::endl
              << "Your input was (" << i << "," << j << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  //if(i>=j){ array[i+n*j] = v; }
  //else{ array[j+n*i] = std::conj(v); }
  if(i>=j){ darray[j][i] = v; }
  else{ darray[i][j] = std::conj(v); }
  
  return *this;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
inline std::ostream& operator<<(std::ostream& s, const zhematrix& mat)
{VERBOSE_REPORT;
  for(long i=0; i<mat.n; i++){
    for(long j=0; j<mat.n; j++){ 
      if(i>j){ s << " " << mat(i,j) << " "; }
      else if(i==j){ s << " " << std::real(mat(i,i)) << " "; }
      else{ s << "{" << std::conj(mat(j,i)) << "} "; }
    }
    s << std::endl;
    
#ifdef  CPPL_DEBUG
    if(std::fabs(std::imag(mat(i,i))) > DBL_MIN){
      WARNING_REPORT;
      std::cerr << "The " << i << "th diagonal component of the zhematrix is not a real number." << std::endl;
    }
#endif//CPPL_DEBUG
  }
  
  return s;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
inline void zhematrix::write(const char* filename) const
{VERBOSE_REPORT;
  std::ofstream ofs(filename, std::ios::trunc);
  ofs.setf(std::cout.flags());
  ofs.precision(std::cout.precision());
  ofs.width(std::cout.width());
  ofs.fill(std::cout.fill());
  
  ofs << "#zhematrix" << " " << n << std::endl;
  for(long i=0; i<n; i++){
    for(long j=0; j<=i; j++ ){
      ofs << operator()(i,j) << " ";
    }
    ofs << std::endl;

#ifdef  CPPL_DEBUG
    if(std::fabs(std::imag(operator()(i,i))) > DBL_MIN){
      WARNING_REPORT;
      std::cerr << "The " << i << "th diagonal component of the zhematrix is not a real number." << std::endl;
    }
#endif//CPPL_DEBUG
  }
  
  ofs.close();
}

//=============================================================================
inline void zhematrix::read(const char* filename)
{VERBOSE_REPORT;
  std::ifstream s(filename);
  if(!s){
    ERROR_REPORT;
    std::cerr << "The file \"" << filename << "\" can not be opened." << std::endl;
    exit(1);
  }

  std::string id;
  s >> id;
  if( id != "zhematrix" && id != "#zhematrix" ){
    ERROR_REPORT;
    std::cerr << "The type name of the file \"" << filename << "\" is not zhematrix." << std::endl
              << "Its type name was " << id << " ." << std::endl;
    exit(1);
  }
  
  s >> n;
  resize(n);
  for(long i=0; i<n; i++){
    for(long j=0; j<=i; j++ ){
      s >> darray[j][i];
      //s >> operator()(i,j); //NG
    }
  }
  if(s.eof()){
    ERROR_REPORT;
    std::cerr << "There is something is wrong with the file \"" << filename << "\"." << std::endl
              << "Most likely, there is not enough data components, or a linefeed code or space code is missing at the end of the last line." << std::endl;
    exit(1);
  }
  
  s >> id;
  if(!s.eof()){
    ERROR_REPORT;
    std::cerr << "There is something is wrong with the file \"" << filename << "\"." << std::endl
              << "Most likely, there are extra data components." << std::endl;
    exit(1);
  }
  
  s.close();
}
