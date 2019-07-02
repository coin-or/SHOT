//=============================================================================
/*! operator() for const object */
inline double _dssmatrix::operator()(const long& i, const long& j) const
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if( i<0 || j<0 || n<=i || n<=j ){
    ERROR_REPORT;
    std::cerr << "The required component is out of the matrix size." << std::endl
              << "Your input was (" << i << "," << j << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  //// search (i,j) component ////
  const uint32_t ii(std::max(i,j)), jj(std::min(i,j));
  for(std::vector<uint32_t>::iterator p=line[ii].begin(); p!=line[ii].end(); p++){
    if(data[*p].i==ii && data[*p].j==jj){ return data[*p].v; }
  }
  
  //// (i,j) component was not found ////
  return 0.0;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
inline std::ostream& operator<<(std::ostream& s, const _dssmatrix& mat)
{VERBOSE_REPORT;
  for(long i=0; i<mat.n; i++){
    for(long j=0; j<mat.n; j++){
      if( i >= j ){
        std::vector<uint32_t>::iterator q;
        for(q=mat.line[i].begin(); q!=mat.line[i].end(); q++){
          if(long(mat.data[*q].j)==j){ break; }
        }
        if(q!=mat.line[i].end()){ s << " " << mat.data[*q].v << " "; }
        else{ s << " x "; }
      }
      else{//i<j
        std::vector<uint32_t>::iterator q;
        for(q=mat.line[i].begin(); q!=mat.line[i].end(); q++){
          if(long(mat.data[*q].j)==j){ break; }
        }
        if(q!=mat.line[i].end()){ s << "{" << mat.data[*q].v << "}"; }
        else{ s << "{x}"; }
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
inline void _dssmatrix::write(const char* filename) const
{VERBOSE_REPORT;
  std::ofstream ofs(filename, std::ios::trunc);
  ofs.setf(std::cout.flags());
  ofs.precision(std::cout.precision());
  ofs.width(std::cout.width());
  ofs.fill(std::cout.fill());
  
  ofs << "#dssmatrix" << std::endl;
  for(std::vector<dcomponent>::const_iterator it=data.begin(); it!=data.end(); it++){
    ofs << it->i << " " << it->j << " " << it->v << std::endl;
  }
  
  ofs.close();
  destroy();
}
