//=============================================================================
/*! operator() for const object */
inline comple _zgsmatrix::operator()(const long& i, const long& j) const
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if( i<0 || j<0 || m<=i || n<=j ){
    ERROR_REPORT;
    std::cerr << "The required component is out of the matrix size." << std::endl
              << "Your input was (" << i << "," << j << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  //////// search (i,j) component ////////
  for(std::vector<uint32_t>::iterator p=rows[i].begin(); p!=rows[i].end(); p++){
    if( long(data[*p].j)==j ){ return data[*p].v; }
  }
  
  //////// (i,j) component was not found ////////
  return comple(0.0,0.0);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
inline std::ostream& operator<<(std::ostream& s, const _zgsmatrix& mat)
{VERBOSE_REPORT;
  for(long i=0; i<mat.m; i++){
    for(long j=0; j<mat.n; j++){
      std::vector<uint32_t>::iterator q;
      for(q=mat.rows[i].begin(); q!=mat.rows[i].end(); q++){
        if( long(mat.data[*q].j)==j ){ break; }
      }
      if(q!=mat.rows[i].end()){ s << " " << mat.data[*q].v; }
      else{ s << " x"; }
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
inline void _zgsmatrix::write(const char* filename) const
{VERBOSE_REPORT;
  std::ofstream ofs(filename, std::ios::trunc);
  ofs.setf(std::cout.flags());
  ofs.precision(std::cout.precision());
  ofs.width(std::cout.width());
  ofs.fill(std::cout.fill());
  
  ofs << "#zgsmatrix" << " " << m << " " << n << std::endl;
  for(std::vector<zcomponent>::const_iterator it=data.begin(); it!=data.end(); it++){
    ofs << it->i << " " << it->j << " " << it->v << std::endl;
  }
  
  ofs.close();
  destroy();
}
