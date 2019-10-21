//=============================================================================
/*! operator() for const object */
inline double& _dgbmatrix::operator()(const long& i, const long& j) const
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
inline std::ostream& operator<<(std::ostream& s, const _dgbmatrix& mat)
{VERBOSE_REPORT;
  for(long i=0; i<mat.m; i++){
    for(long j=0; j<mat.n; j++){
      if( i-j>mat.kl || j-i>mat.ku ){ s << " x"; }
      else{ s << " " << mat(i,j); }
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
inline void _dgbmatrix::write(const char *filename) const
{VERBOSE_REPORT;
  std::ofstream ofs(filename, std::ios::trunc);
  ofs.setf(std::cout.flags());
  ofs.precision(std::cout.precision());
  ofs.width(std::cout.width());
  ofs.fill(std::cout.fill());
  
  ofs << "#dgbmatrix" << " " << m << " " << n << " " << kl << " " << ku << std::endl;
  for(long i=0; i<m; i++){
    for(long j=std::max(long(0),i-kl); j<std::min(n,i+ku+1); j++){
      ofs << operator()(i,j) << " ";
    }
    ofs << std::endl;
  }
  
  ofs.close();
  destroy();
}
