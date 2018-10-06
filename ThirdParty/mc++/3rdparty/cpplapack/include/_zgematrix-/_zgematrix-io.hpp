//=============================================================================
/*! operator() for object */
inline comple& _zgematrix::operator()(const long& i, const long& j) const
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
inline std::ostream& operator<<(std::ostream& s, const _zgematrix& mat)
{VERBOSE_REPORT;
  for(long i=0; i<mat.m; i++){
    for(long j=0; j<mat.n; j++){
      s << " " << mat(i,j);
    }
    s << std::endl;
  }
  
  mat.destroy();
  return s;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
inline void _zgematrix::write(const char *filename) const
{VERBOSE_REPORT;
  std::ofstream ofs(filename, std::ios::trunc);
  ofs.setf(std::cout.flags());
  ofs.precision(std::cout.precision());
  ofs.width(std::cout.width());
  ofs.fill(std::cout.fill());
  
  ofs << "#zgematrix" << " " << m << " " << n << std::endl;
  for(long i=0; i<m; i++){
    for(long j=0; j<n; j++){
      ofs << operator()(i,j) << " ";
    }
    ofs << std::endl;
  }
  
  ofs.close();
  destroy();
}
