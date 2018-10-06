//=============================================================================
/*! operator() for non-const object */
inline double& dgematrix::operator()(const long& i, const long& j)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if( i<0 || j<0 || m<=i || n<=j ){
    ERROR_REPORT;
    std::cerr << "The required component is out of the matrix size." << std::endl
              << "Your input was (" << i << "," << j << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  //return array[i+m*j];
  return darray[j][i];
}

//=============================================================================
/*! operator() for const object */
inline double dgematrix::operator()(const long& i, const long& j) const
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if( i<0 || j<0 || m<=i || n<=j ){
    ERROR_REPORT;
    std::cerr << "The required component is out of the matrix size." << std::endl
              << "Your input was (" << i << "," << j << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  //return array[i+m*j];
  return darray[j][i];
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! set value for const object */
inline dgematrix& dgematrix::set(const long& i, const long& j, const double& v) //const
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if( i<0 || j<0 || m<=i || n<=j ){
    ERROR_REPORT;
    std::cerr << "The required component is out of the matrix size." << std::endl
              << "Your input was (" << i << "," << j << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  //array[i+m*j] =v;
  darray[j][i] =v;
  
  return *this;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
inline std::ostream& operator<<(std::ostream& s, const dgematrix& mat)
{VERBOSE_REPORT;
  for(long i=0; i<mat.m; i++){
    for(long j=0; j<mat.n; j++){
      s << " " << mat(i,j);
    }
    s << std::endl;
  }
  return s;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
inline void dgematrix::write(const char* filename) const
{VERBOSE_REPORT;
  std::ofstream ofs(filename, std::ios::trunc);
  ofs.setf(std::cout.flags());
  ofs.precision(std::cout.precision());
  ofs.width(std::cout.width());
  ofs.fill(std::cout.fill());
  
  ofs << "#dgematrix" << " " << m << " " << n << std::endl;
  for(long i=0; i<m; i++){
    for(long j=0; j<n; j++ ){
      ofs << operator()(i,j) << " ";
    }
    ofs << std::endl;
  }
  
  ofs.close();
}

//=============================================================================
inline void dgematrix::read(const char* filename)
{VERBOSE_REPORT;
  std::ifstream s( filename );
  if(!s){
    ERROR_REPORT;
    std::cerr << "The file \"" << filename << "\" can not be opened." << std::endl;
    exit(1);
  }

  std::string id;
  s >> id;
  if( id != "dgematrix" && id != "#dgematrix" ){
    ERROR_REPORT;
    std::cerr << "The type name of the file \"" << filename << "\" is not dgematrix." << std::endl
              << "Its type name was " << id << " ." << std::endl;
    exit(1);
  }
  
  s >> m >> n;
  resize(m, n);
  for(long i=0; i<m; i++){
    for(long j=0; j<n; j++ ){
      s >> operator()(i,j);
    }
  }
  if(s.eof()){
    ERROR_REPORT;
    std::cerr << "There is something is wrong with the file \"" << filename << "\"." << std::endl
              << "Most likely, there is a lack of data components, or a linefeed code or space code is missing at the end of the last line." << std::endl;
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
