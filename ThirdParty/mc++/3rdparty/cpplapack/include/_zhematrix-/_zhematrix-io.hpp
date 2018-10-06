//=============================================================================
/*! operator() for object */
inline zhecomplex _zhematrix::operator()(const long& i, const long& j) const
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

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
inline std::ostream& operator<<(std::ostream& s, const _zhematrix& mat)
{VERBOSE_REPORT;
  for(long i=0; i<mat.n; i++){
    for(long j=0; j<mat.n; j++){
      if(i>j){ s << " " << mat(i,j) << " "; }
      else if(i==j){ s << " " << std::real(mat(i,i)) << " "; }
      else{ s << "{" << std::conj(mat.darray[i][j]) << "} "; }
    }
    s << std::endl;
    
#ifdef  CPPL_DEBUG
    if(std::fabs(std::imag(mat(i,i))) > DBL_MIN){
      WARNING_REPORT;
      std::cerr << "The " << i << "th diagonal component of the zhematrix is not a real number." << std::endl;
    }
#endif//CPPL_DEBUG
  }
  
  mat.destroy();
  return s;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
inline void _zhematrix::write(const char* filename) const
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
  destroy();
}
