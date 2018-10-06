//=============================================================================
/*! operator() for const object */
inline double dgsmatrix::operator()(const long& i, const long& j) const
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if( i<0 || j<0 || m<=i || n<=j ){
    ERROR_REPORT;
    std::cerr << "The required component is out of the matrix size." << std::endl
              << "Your input was (" << i << "," << j << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  //// search (i,j) component ////
  for(std::vector<uint32_t>::const_iterator p=rows[i].begin(); p!=rows[i].end(); p++){
    if(long(data[*p].j)==j){ return data[*p].v; }
  }
  
  //// (i,j) component was not found ////
  return 0.0;
}

//=============================================================================
/*! operator() for const object */
inline double& dgsmatrix::operator()(const long& i, const long& j)
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
    if(long(data[*p].j)==j){ return data[*p].v; }
  }
  
  //////// (i,j) component not found ////////
  rows[i].push_back(data.size());
  cols[j].push_back(data.size());
  data.push_back(dcomponent(i,j,0.));
  return data.back().v;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! put value with volume cheack without isListed check */
inline dgsmatrix& dgsmatrix::put(const long& i, const long& j, const double& v)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if( i<0 || j<0 || m<=i || n<=j ){
    ERROR_REPORT;
    std::cerr << "The required component is out of the matrix size." << std::endl
              << "Your input was (" << i << "," << j << ")." << std::endl;
    exit(1);
  }
  for(std::vector<dcomponent>::const_iterator it=data.begin(); it!=data.end(); it++){
    if( long(it->i)==i && long(it->j)==j ){
      ERROR_REPORT;
      std::cerr << "The required component is already listed." << std::endl
                << "Your input was (" << i << "," << j << "," << v << ")." << std::endl;
      exit(1);
    }
  }
#endif//CPPL_DEBUG
  
  //// push ////
  rows[i].push_back(data.size());
  cols[j].push_back(data.size());
  data.push_back(dcomponent(i,j,v));
  
  return *this;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! delete the entry of a component */
inline dgsmatrix& dgsmatrix::del(const long i, const long j)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if( i<0 || j<0 || m<=i || n<=j ){
    ERROR_REPORT;
    std::cerr << "The required component is out of the matrix size." << std::endl
              << "Your input was (" << i << "," << j << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  //// search (i,j) component ////
  for(std::vector<uint32_t>::iterator p=rows[i].begin(); p!=rows[i].end(); p++){
    if(long(data[*p].j)==j){//exists
      //// save position ////
      uint32_t c(*p);
      uint32_t C(data.size()-1);
      
      //// data translation ////
      data[c]=data.back();
      data.pop_back();
      
      //// remove from List ////
      rows[i].erase(p);
      for(std::vector<uint32_t>::iterator q=cols[j].begin(); q!=cols[j].end(); q++){
        if(*q==c){ cols[j].erase(q); break; }
      }
      
      //// modify the entry of translated component ////
      long I(data[c].i), J(data[c].j);
      for(std::vector<uint32_t>::iterator q=rows[I].begin(); q!=rows[I].end(); q++){
        if(*q==C){ *q=c; break; }
      }
      for(std::vector<uint32_t>::iterator q=cols[J].begin(); q!=cols[J].end(); q++){
        if(*q==C){ *q=c; break; }
      }
      return *this;
    }
  }
  
#ifdef  CPPL_DEBUG
  std::cerr << "# [NOTE]@dgsmatrix::del(long&, long&): The required component was not listed. Your input was (" << i << "," << j << ")." << std::endl;
#endif//CPPL_DEBUG
  
  return *this;
}

//=============================================================================
/*! delete the entry of an element */
inline dgsmatrix& dgsmatrix::del(const long c)
{VERBOSE_REPORT;
  if( c==long(data.size()-1) ){//if c is the last element
    long i(data[c].i), j(data[c].j);
    for(std::vector<uint32_t>::iterator q=rows[i].begin(); q!=rows[i].end(); q++){
      if(long(*q)==c){ rows[i].erase(q); break; }
    }
    for(std::vector<uint32_t>::iterator q=cols[j].begin(); q!=cols[j].end(); q++){
      if(long(*q)==c){ cols[j].erase(q); break; }
    }
    data.pop_back();
  }
  
  else{//if c is NOT the last element
    //// data translation ////
    uint32_t C(data.size()-1);
    long i(data[c].i), j(data[c].j), I(data.back().i), J(data.back().j);
    data[c]=data.back();
    //// remove entry of component ////
    for(std::vector<uint32_t>::iterator q=rows[i].begin(); q!=rows[i].end(); q++){
      if(long(*q)==c){ rows[i].erase(q); break; }
    }
    for(std::vector<uint32_t>::iterator q=cols[j].begin(); q!=cols[j].end(); q++){
      if(long(*q)==c){ cols[j].erase(q); break; }
    }
    //// modify the entry of translated component ////
    for(std::vector<uint32_t>::iterator q=rows[I].begin(); q!=rows[I].end(); q++){
      if(*q==C){ *q=c; break; }
    }
    for(std::vector<uint32_t>::iterator q=cols[J].begin(); q!=cols[J].end(); q++){
      if(*q==C){ *q=c; break; }
    }
    //// pop_back ////
    data.pop_back();
  }
  
  return *this;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
inline std::ostream& operator<<(std::ostream& s, const dgsmatrix& mat)
{VERBOSE_REPORT;
  for(long i=0; i<mat.m; i++){
    for(long j=0; j<mat.n; j++){
      std::vector<uint32_t>::const_iterator q;
      for(q=mat.rows[i].begin(); q!=mat.rows[i].end(); q++){
        if(long(mat.data[*q].j)==j){ break; }
      }
      if(q!=mat.rows[i].end()){ s << " " << mat.data[*q].v; }
      else{ s << " x"; }
    }
    s << std::endl;
  }
  
  return s;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
inline void dgsmatrix::write(const char* filename) const
{VERBOSE_REPORT;
  std::ofstream ofs(filename, std::ios::trunc);
  ofs.setf(std::cout.flags());
  ofs.precision(std::cout.precision());
  ofs.width(std::cout.width());
  ofs.fill(std::cout.fill());
  
  ofs << "#dgsmatrix" << " " << m << " " << n << std::endl;
  for(std::vector<dcomponent>::const_iterator it=data.begin(); it!=data.end(); it++){
    ofs << it->i << " " << it->j << " " << it->v << std::endl;
  }
  
  ofs.close();
}

//=============================================================================
inline void dgsmatrix::read(const char* filename)
{VERBOSE_REPORT;
  std::ifstream s( filename );
  if(!s){
    ERROR_REPORT;
    std::cerr << "The file \"" << filename << "\" can not be opened." << std::endl;
    exit(1);
  }

  std::string id;
  s >> id;
  if( id != "dgsmatrix" && id != "#dgsmatrix" ){
    ERROR_REPORT;
    std::cerr << "The type name of the file \"" << filename << "\" is not dgsmatrix." << std::endl
              << "Its type name was " << id << " ." << std::endl;
    exit(1);
  }
  
  s >> m >> n;
  resize(m, n);
  
  double val;
  long i, j,  pos, tmp;
  while(!s.eof()){
    s >> i >> j >> val;
    put(i,j, val);
    pos =s.tellg();
    s >> tmp;
    s.seekg(pos);
  }
  
  if(!s.eof()){
    ERROR_REPORT;
    std::cerr << "There is something is wrong with the file \"" << filename << " ." << std::endl
              << "Most likely, there are too many data components over the context." << std::endl;
    exit(1);
  }
  s.close();
}
