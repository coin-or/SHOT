//=============================================================================
/*! operator() for const object */
inline double& _dcovector::operator()(const long& i) const
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if( i<0 || l<=i ){
    ERROR_REPORT;
    std::cerr << "The required component is out of the vector size." << std::endl
              << "Your input was (" << i << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  return array[i];
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
inline std::ostream& operator<<(std::ostream& s, const _dcovector& vec)
{VERBOSE_REPORT;
  for(long i=0; i<vec.l; i++){
    s << " " << vec.array[i] << std::endl;
  }
  
  vec.destroy();
  return s;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
inline void _dcovector::write(const char *filename) const
{VERBOSE_REPORT;
  std::ofstream ofs(filename, std::ios::trunc);
  ofs.setf(std::cout.flags());
  ofs.precision(std::cout.precision());
  ofs.width(std::cout.width());
  ofs.fill(std::cout.fill());
  
  ofs << "#dcovector" << " " << l << std::endl;
  for(long i=0; i<l; i++){
    ofs << operator()(i) << std::endl;
  }
  
  ofs.close();
  destroy();
}
