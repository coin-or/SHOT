//=============================================================================
/*! operator() for const object */
inline comple zhsmatrix::operator()(const long& i, const long& j) const
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
  for(std::vector<uint32_t>::const_iterator p=line[ii].begin(); p!=line[ii].end(); p++){
    if(data[*p].j==jj){
      if( i>j ){ return data[*p].v; }//ii=i
      else{      return std::conj(data[*p].v); }//ii=j
    }
  }
  
  //// (i,j) component was not found ////
  return comple(0.0,0.0);
}

//=============================================================================
/*! operator() */
inline zhecomplex zhsmatrix::operator()(const long& i, const long& j)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if( i<0 || j<0 || n<=i || n<=j ){
    ERROR_REPORT;
    std::cerr << "The required component is out of the matrix size." << std::endl
              << "Your input was (" << i << "," << j << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  //////// search (i,j) component ////////
  const uint32_t ii(std::max(i,j)), jj(std::min(i,j));
  for(std::vector<uint32_t>::const_iterator p=line[ii].begin(); p!=line[ii].end(); p++){
    if(data[*p].j==jj){
      return zhecomplex(i,j, data[*p].v);
    }
  }
  
  //////// (i,j) component not found ////////
  line[i].push_back(data.size());
  if(i!=j){//off-diagonal
    line[j].push_back(data.size());
  }
  data.push_back(zcomponent(ii,jj,comple(0.,0.)));
  return zhecomplex(i,j, data.back().v);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! put value with volume cheack without isListed check */
inline zhsmatrix& zhsmatrix::put(const long& i, const long& j, const comple& v)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if( i<0 || j<0 || n<=i || n<=j ){
    ERROR_REPORT;
    std::cerr << "The required component is out of the matrix size." << std::endl
              << "Your input was (" << i << "," << j << ")." << std::endl;
    exit(1);
  }
  
  if( isListed(i,j) ){
    ERROR_REPORT;
    std::cerr << "The required component is already listed." << std::endl
              << "Your input was (" << i << "," << j << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  //// push ////
  const long ii(std::max(i,j)), jj(std::min(i,j));
  line[ii].push_back(data.size());
  if(i!=j){//off-diagonal
    line[jj].push_back(data.size());
  }
  data.push_back(zcomponent(ii,jj,v));
  return *this;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! delete the entry of a component */
inline zhsmatrix& zhsmatrix::del(const long i, const long j)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if( i<0 || j<0 || n<=i || n<=j ){
    ERROR_REPORT;
    std::cerr << "The required component is out of the matrix size." << std::endl
              << "Your input was (" << i << "," << j << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG

  const long ii(std::max(i,j)), jj(std::min(i,j));

  //////// search (i,j) component ////////
  for(std::vector<uint32_t>::iterator p=line[ii].begin(); p!=line[ii].end(); p++){
    if( long(data[*p].i)==ii && long(data[*p].j)==jj ){//exists
      //// save position ////
      uint32_t c(*p);
      uint32_t C(data.size()-1);

      //// data translation ////
      data[c]=data.back();
      data.pop_back();

      //// remove from List ////
      line[ii].erase(p);
      if(i!=j){//off-diagonal
        for(std::vector<uint32_t>::iterator pj=line[jj].begin(); pj!=line[jj].end(); pj++){
          if(*pj==c){ line[jj].erase(pj); break; }
        }
      }

      //// modify the entry of translated component ////
      uint32_t I(data[c].i), J(data[c].j);
      for(std::vector<uint32_t>::iterator q=line[I].begin(); q!=line[I].end(); q++){
        if(*q==C){ *q=c; break; }
      }
      if(I!=J){//off-diagonal
        for(std::vector<uint32_t>::iterator q=line[J].begin(); q!=line[J].end(); q++){
          if(*q==C){ *q=c; break; }
        }
      }
      return *this;
    }
  }
  
#ifdef  CPPL_DEBUG
  std::cerr << "# [NOTE]@zhsmatrix::del(long&, long&): The required component was not listed. Your input was (" << i << "," << j << ")." << std::endl;
#endif//CPPL_DEBUG
  
  return *this;
}

//=============================================================================
/*! delete the entry of an element */
inline zhsmatrix& zhsmatrix::del(const long c)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if( c<0 || c>=long(data.size()) ){
    ERROR_REPORT;
    std::cerr << "The required element is out of the matrix volume." << std::endl
              << "Your input was (" << c << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  if( c==long(data.size()-1) ){//if c is the last element
    long i(data[c].i), j(data[c].j);
    for(std::vector<uint32_t>::iterator q=line[i].begin(); q!=line[i].end(); q++){
      if( long(*q)==c ){ line[i].erase(q); break; }
    }
    if(i!=j){//off-diagonal
      for(std::vector<uint32_t>::iterator q=line[j].begin(); q!=line[j].end(); q++){
        if( long(*q)==c ){ line[j].erase(q); break; }
      }
    }
    data.pop_back();
  }
  
  else{//c is NOT the last element
    //// data translation ////
    uint32_t C(data.size()-1);
    long i(data[c].i), j(data[c].j), I(data.back().i), J(data.back().j);
    data[c]=data.back();
    //std::cerr << "c=" << c   << " i=" << i << " j=" << j << " C=" << vol << " I=" << I << " J=" << J << std::endl;
    //// remove entry of component ////
    for(std::vector<uint32_t>::iterator q=line[i].begin(); q!=line[i].end(); q++){
      if( long(*q)==c ){ line[i].erase(q); break; }
    }
    if(i!=j){//off-diagonal
      for(std::vector<uint32_t>::iterator q=line[j].begin(); q!=line[j].end(); q++){
        if( long(*q)==c ){ line[j].erase(q); break; }
      }
    }
    //// modify the entry of translated component ////
    for(std::vector<uint32_t>::iterator q=line[I].begin(); q!=line[I].end(); q++){
      if(*q==C){ *q=c; break; }
    }
    if(I!=J){//off-diagonal
      for(std::vector<uint32_t>::iterator q=line[J].begin(); q!=line[J].end(); q++){
        if(*q==C){ *q=c; break; }
      }
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
inline std::ostream& operator<<(std::ostream& s, const zhsmatrix& mat)
{VERBOSE_REPORT;
  for(long i=0; i<mat.n; i++){
    for(long j=0; j<mat.n; j++){
      if( i >= j ){
        long c =mat.number(i,j);
        if(c<0){
          s << " x ";
        }
        else{
          s << " " << mat.data[c].v << " ";
        }
      }
      else{//i<j
        long c =mat.number(i,j);
        if(c<0){
          s << "{x}";
        }
        else{
          s << "{" << std::conj(mat.data[c].v) << "}";
        }
      }
    }
    s << std::endl;
  }
  
  return s;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
inline void zhsmatrix::write(const char* filename) const
{VERBOSE_REPORT;
  std::ofstream ofs(filename, std::ios::trunc);
  ofs.setf(std::cout.flags());
  ofs.precision(std::cout.precision());
  ofs.width(std::cout.width());
  ofs.fill(std::cout.fill());
  
  ofs << "#zhsmatrix" << " " << n << std::endl;
  for(std::vector<zcomponent>::const_iterator it=data.begin(); it!=data.end(); it++){
    ofs << it->i << " " << it->j << " " << it->v << std::endl;
  }
  ofs.close();
}

//=============================================================================
inline void zhsmatrix::read(const char* filename)
{VERBOSE_REPORT;
  std::ifstream s( filename );
  if(!s){
    ERROR_REPORT;
    std::cerr << "The file \"" << filename << "\" can not be opened." << std::endl;
    exit(1);
  }

  std::string id;
  s >> id;
  if( id != "zhsmatrix" && id != "#zhsmatrix" ){
    ERROR_REPORT;
    std::cerr << "The type name of the file \"" << filename << "\" is not zhsmatrix." << std::endl
              << "Its type name was " << id << " ." << std::endl;
    exit(1);
  }
  
  s >> n;
  resize(n);
  
  comple val;
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
