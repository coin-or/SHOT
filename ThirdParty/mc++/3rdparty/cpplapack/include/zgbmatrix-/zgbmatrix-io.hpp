//=============================================================================
/*! operator() for non-const object */
inline comple& zgbmatrix::operator()(const long& i, const long& j)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if( i<0 || j<0 || m<=i || n<=j || i-j>kl || j-i>ku ){
    ERROR_REPORT;
    std::cerr << "The required component is out of the matrix size." << std::endl
              << "Your input was (" << i << "," << j << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  //return array[ku+i+(kl+ku)*j];
  return darray[j][ku-j+i];
}

//=============================================================================
/*! operator() for const object */
inline comple zgbmatrix::operator()(const long& i, const long& j) const
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if( i<0 || j<0 || m<=i || n<=j || i-j>kl || j-i>ku ){
    ERROR_REPORT;
    std::cerr << "The required component is out of the matrix size." << std::endl
              << "Your input was (" << i << "," << j << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  //return array[ku+i+(kl+ku)*j];
  return darray[j][ku-j+i];
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! set value for const object */
inline zgbmatrix& zgbmatrix::set(const long& i, const long& j, const comple& v)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if( i<0 || j<0 || m<=i || n<=j || i-j>kl || j-i>ku ){
    ERROR_REPORT;
    std::cerr << "The required component is out of the matrix size." << std::endl
              << "Your input was (" << i << "," << j << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  //array[ku+i+(kl+ku)*j] =v;
  darray[j][ku-j+i] =v;
  
  return *this;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
inline std::ostream& operator<<(std::ostream& s, const zgbmatrix& mat)
{VERBOSE_REPORT;
  for(long i=0; i<mat.m; i++){
    for(long j=0; j<mat.n; j++){
      if( i-j>mat.kl || j-i>mat.ku ){ s << " x"; }
      else{ s << " " << mat(i,j); }
    }
    s << std::endl;
  }
  
  return s;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
inline void zgbmatrix::write(const char* filename) const
{VERBOSE_REPORT;
  std::ofstream ofs(filename, std::ios::trunc);
  ofs.setf(std::cout.flags());
  ofs.precision(std::cout.precision());
  ofs.width(std::cout.width());
  ofs.fill(std::cout.fill());
  
  ofs << "#zgbmatrix" << " " << m << " " << n << " " << kl << " " << ku << std::endl;
  for(long i=0; i<m; i++){
    for(long j=std::max(long(0),i-kl); j<std::min(n,i+ku+1); j++){
      ofs << operator()(i,j) << " ";
    }
    ofs << std::endl;
  }
  
  ofs.close();
}

//=============================================================================
inline void zgbmatrix::read(const char* filename)
{VERBOSE_REPORT;
  std::ifstream s( filename );
  if(!s){
    ERROR_REPORT;
    std::cerr << "The file \"" << filename << "\" can not be opened." << std::endl;
    exit(1);
  }

  std::string id;
  s >> id;
  if( id != "zgbmatrix" && id != "#zgbmatrix" ){
    ERROR_REPORT;
    std::cerr << "The type name of the file \"" << filename << "\" is not zgbmatrix." << std::endl
              << "Its type name was " << id << " ." << std::endl;
    exit(1);
  }
  
  s >> m >> n >> kl >> ku;
  resize(m, n, kl, ku);
  for(long i=0; i<m; i++){
    for(long j=std::max(long(0),i-kl); j<std::min(n,i+ku+1); j++){
      s >> operator()(i,j);
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
