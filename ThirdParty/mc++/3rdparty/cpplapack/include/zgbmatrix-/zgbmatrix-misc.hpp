//=============================================================================
/*! clear all the matrix data and set the sizes 0 */
inline void zgbmatrix::clear()
{VERBOSE_REPORT;
  m =0;
  n =0;
  kl =0;
  ku =0;
  delete [] array;
  array =NULL;
  delete [] darray;
  darray =NULL;
}


//=============================================================================
/*! change the matrix into a zero matrix */
inline zgbmatrix& zgbmatrix::zero()
{VERBOSE_REPORT;
  for(long i=0; i<(kl+ku+1)*n; i++){
    array[i] =comple(0.0,0.0);
  }
  return *this;
}

//=============================================================================
/*! change the matrix into an identity matrix */
inline zgbmatrix& zgbmatrix::identity()
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(m!=n){
    ERROR_REPORT;
    std::cerr << "Only square matrix can be a identity matrix." << std::endl
              << "The matrix size was " << m << "x" << n << "." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  for(long i=0; i<(kl+ku+1)*n; i++){ array[i] =comple(0.0,0.0); }
  for(long i=0; i<m; i++){ operator()(i,i) =comple(1.0,0.0); }
  
  return *this;
}

//=============================================================================
/*! change sign(+/-) of the matrix */
inline void zgbmatrix::chsign()
{VERBOSE_REPORT;
  for(long i=0; i<(kl+ku+1)*n; i++){ array[i] =-array[i]; }
}

//=============================================================================
/*! make a deep copy of the matrix */
inline void zgbmatrix::copy(const zgbmatrix& mat)
{VERBOSE_REPORT;
  m =mat.m;
  n =mat.n;
  kl =mat.kl;
  ku =mat.ku;
  delete [] array;
  array =new comple[(mat.kl+mat.ku+1)*mat.n];
  delete [] darray;
  darray =new comple*[n];
  for(int i=0; i<n; i++){ darray[i] =&array[i*(kl+ku+1)]; }
  
  zcopy_((mat.kl+mat.ku+1)*mat.n, mat.array, 1, array, 1);
}

//=============================================================================
/*! make a shallow copy of the matrix\n
  This function is not designed to be used in project codes. */
inline void zgbmatrix::shallow_copy(const _zgbmatrix& mat)
{VERBOSE_REPORT;
  m =mat.m;
  n =mat.n;
  kl =mat.kl;
  ku =mat.ku;
  delete [] array;
  array =mat.array;
  delete [] darray;
  darray =mat.darray;
  
  mat.nullify();
}

//=============================================================================
/*! resize the matrix */
inline void zgbmatrix::resize(const long& _m, const long& _n,
                              const long& _kl, const long& _ku)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if( _m<0 || _n<0 || _kl<0 || _ku<0 || _m<_kl || _n<_ku ){
    ERROR_REPORT;
    std::cerr << "It is impossible to make a matrix you ordered. " << std::endl
              << "Your input was (" << _m << "," << _n << ","<< _ku << "," << _kl << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  m =_m;
  n =_n;
  kl =_kl;
  ku =_ku;
  delete [] array;
  array =new comple[(kl+ku+1)*n];
  delete [] darray;
  darray =new comple*[n];
  for(int i=0; i<n; i++){ darray[i] =&array[i*(kl+ku+1)]; }
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! get row of the matrix */
inline _zrovector zgbmatrix::row(const long& _m) const
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if( _m<0 || _m>m ){
    ERROR_REPORT;
    std::cerr << "Input row number must be between 0 and " << m << "." << std::endl
              << "Your input was " << _m << "." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  zrovector v( zrovector(n).zero() );
  for(long j=std::max(long(0),_m-kl); j<std::min(n,_m+ku+1); j++){ v(j)=(*this)(_m,j); }
  return _(v);
}

//=============================================================================
/*! get column of the matrix */
inline _zcovector zgbmatrix::col(const long& _n) const
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if( _n<0 || _n>n ){
    ERROR_REPORT;
    std::cerr << "Input row number must be between 0 and " << n << "." << std::endl
              << "Your input was " << _n << "." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  zcovector v( zcovector(m).zero() );
  for(long i=std::max(long(0),_n-ku); i<std::min(m,_n+kl+1); i++){ v(i)=(*this)(i,_n); }
  return _(v);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! swap two matrices */
inline void swap(zgbmatrix& A, zgbmatrix& B)
{VERBOSE_REPORT;
  long A_m(A.m), A_n(A.n), A_kl(A.kl), A_ku(A.ku);
  comple* A_array(A.array);
  //comple** A_darray(A.darray);
  comple** A_darray=A.darray; //corruption to support VC++

  A.m=B.m; A.n=B.n; A.kl=B.kl; A.ku=B.ku; A.array=B.array; A.darray=B.darray;
  B.m=A_m; B.n=A_n; B.kl=A_kl; B.ku=A_ku; B.array=A_array; B.darray=A_darray;
}

//=============================================================================
/*! convert user object to smart-temporary object */
inline _zgbmatrix _(zgbmatrix& mat)
{VERBOSE_REPORT;
  _zgbmatrix newmat;
  
  //////// shallow copy ////////
  newmat.m =mat.m;
  newmat.n =mat.n;
  newmat.kl =mat.kl;
  newmat.ku =mat.ku;
  newmat.array =mat.array;
  newmat.darray =mat.darray;
  
  //////// nullify ////////
  mat.m =0;
  mat.n =0;
  mat.kl =0;
  mat.ku =0;
  mat.array =NULL;
  mat.darray =NULL;
  
  return newmat;
}
