//=============================================================================
/*! operator() for object */
inline double& _dsymatrix::operator()(const long& i, const long& j) const
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if( i<0 || j<0 || n<=i || n<=j ){
    ERROR_REPORT;
    std::cerr << "The required component is out of the matrix size." << std::endl
              << "Your input was (" << i << "," << j << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  if( i >= j ){
    //return array[i+n*j];
    return darray[j][i];
  } else {
    //return array[j+n*i];
    return darray[i][j];
  }
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
inline std::ostream& operator<<(std::ostream& s, const _dsymatrix& mat)
{VERBOSE_REPORT;
  for(long i=0; i<mat.n; i++){
    for(long j=0; j<mat.n; j++){
      if( i >= j ){
        s << " " << mat(i,j) << "  ";
      } else {
        s << "{" << mat(i,j) << "} ";
      }
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
inline void _dsymatrix::write(const char* filename) const
{VERBOSE_REPORT;
  std::ofstream ofs(filename, std::ios::trunc);
  ofs.setf(std::cout.flags());
  ofs.precision(std::cout.precision());
  ofs.width(std::cout.width());
  ofs.fill(std::cout.fill());
  
  ofs << "#dsymatrix" << " " << n << std::endl;
  for(long i=0; i<n; i++){
    for(long j=0; j<=i; j++ ){
      ofs << operator()(i,j) << " ";
    }
    ofs << std::endl;
  }
  
  ofs.close();
  destroy();
}
