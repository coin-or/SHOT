//=============================================================================
/*! zgsmatrix=zgsmatrix operator */
inline zgsmatrix& zgsmatrix::operator=(const zgsmatrix& mat)
{VERBOSE_REPORT;
  if(&data!=&mat.data){ // if it is NOT self substitution
    copy(mat);
  }
  return *this;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! zgsmatrix+=zgsmatrix operator */
inline zgsmatrix& zgsmatrix::operator+=(const zgsmatrix& mat)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(n!=mat.n || m!=mat.m){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a summation." << std::endl
              << "Your input was (" << m << "x" << n << ") += (" << mat.m << "x" << mat.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  for(std::vector<zcomponent>::const_iterator it=mat.data.begin(); it!=mat.data.end(); it++){
    (*this)(it->i,it->j) +=it->v;
  }
  return *this;
}

//=============================================================================
/*! zgsmatrix-=zgsmatrix operator */
inline zgsmatrix& zgsmatrix::operator-=(const zgsmatrix& mat)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(n!=mat.n || m!=mat.m){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a sutraction." << std::endl
              << "Your input was (" << m << "x" << n << ") -= (" << mat.m << "x" << mat.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  for(std::vector<zcomponent>::const_iterator it=mat.data.begin(); it!=mat.data.end(); it++){
    (*this)(it->i,it->j) -=it->v;
  }
  return *this;
}

//=============================================================================
/*! zgsmatrix*=zgsmatrix operator */
inline zgsmatrix& zgsmatrix::operator*=(const zgsmatrix& mat)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(n!=mat.m){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a product." << std::endl
              << "Your input was (" << m << "x" << n << ") *= (" << mat.m << "x" << mat.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  zgsmatrix newmat(m, mat.n);
  
  for(std::vector<zcomponent>::const_iterator it=data.begin(); it!=data.end(); it++){
    long k(it->j);
    std::vector<uint32_t>::const_iterator p;
    for(p=mat.rows[k].begin(); p!=mat.rows[k].end(); p++){
      newmat(it->i,mat.data[*p].j) +=it->v*mat.data[*p].v;
    }
  }
  
  swap(*this,newmat);
  return *this;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! zgsmatrix+zgsmatrix operator */
inline _zgsmatrix operator+(const zgsmatrix& matA, const zgsmatrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.n || matA.m!=matB.m){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a summation." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") + (" << matB.m << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  zgsmatrix newmat(matA);
  for(std::vector<zcomponent>::const_iterator it=matB.data.begin(); it!=matB.data.end(); it++){
    newmat(it->i, it->j) +=it->v;
  }
  return _(newmat);
}

//=============================================================================
/*! zgsmatrix-zgsmatrix operator */
inline _zgsmatrix operator-(const zgsmatrix& matA, const zgsmatrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.n || matA.m!=matB.m){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a subtraction." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") - (" << matB.m << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG

  zgsmatrix newmat(matA);
  for(std::vector<zcomponent>::const_iterator it=matB.data.begin(); it!=matB.data.end(); it++){
    newmat(it->i, it->j) -=it->v;
  }
  return _(newmat);
}

//=============================================================================
/*! zgsmatrix*zgsmatrix operator */
inline _zgsmatrix operator*(const zgsmatrix& matA, const zgsmatrix& matB)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(matA.n!=matB.m){
    ERROR_REPORT;
    std::cerr << "These two matrises can not make a product." << std::endl
              << "Your input was (" << matA.m << "x" << matA.n << ") * (" << matB.m << "x" << matB.n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  zgsmatrix newmat(matA.m, matB.n);
  
  for(std::vector<zcomponent>::const_iterator it=matA.data.begin(); it!=matA.data.end(); it++){
    long k(it->j);
    std::vector<uint32_t>::const_iterator p;
    for(p=matB.rows[k].begin(); p!=matB.rows[k].end(); p++){
      newmat(it->i,matB.data[*p].j) +=it->v*matB.data[*p].v;
    }
  }
  
  return _(newmat);
}
