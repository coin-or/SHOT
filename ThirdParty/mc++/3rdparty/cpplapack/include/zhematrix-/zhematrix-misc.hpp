//=============================================================================
/*! complete the upper-right components */
inline void zhematrix::complete() const
{VERBOSE_REPORT;
  for(long i=0; i<n; i++){
    for(long j=0; j<i; j++){
      darray[i][j] =std::conj(darray[j][i]);
    }    
#ifdef  CPPL_DEBUG
    if(std::fabs(std::imag(operator()(i,i))) > DBL_MIN){
      WARNING_REPORT;
      std::cerr << "The " << i << "th diagonal component of the zhematrix is not a real number." << std::endl;
    }
#endif//CPPL_DEBUG
  }
}

//=============================================================================
/*! clear all the matrix data and set the sizes 0 */
inline void zhematrix::clear()
{VERBOSE_REPORT;
  n =0;
  delete [] array;
  array =NULL;
  delete [] darray;
  darray =NULL;
}

//=============================================================================
/*! change the matrix into a zero matrix */
inline zhematrix& zhematrix::zero()
{VERBOSE_REPORT;
  for(long i=0; i<n; i++){
    for(long j=0; j<=i; j++){
      darray[j][i]=comple(0.0,0.0);
    }
  }
  return *this;
}

//=============================================================================
/*! change the matrix into an identity matrix */
inline zhematrix& zhematrix::identity()
{VERBOSE_REPORT;
  for(long i=0; i<n; i++){
    for(long j=0; j<i; j++){
      darray[j][i] =comple(0.0,0.0);
    }
    darray[i][i] =comple(1.0,0.0);
  }
  
  return *this;
}

//=============================================================================
/*! change sign(+/-) of the matrix */
inline void zhematrix::chsign()
{VERBOSE_REPORT;
  for(long i=0; i<n; i++){
    for(long j=0; j<=i; j++){
      darray[j][i] =-darray[j][i];
    }
  }
}

//=============================================================================
/*! make a deep copy of the matrix */
inline void zhematrix::copy(const zhematrix& mat)
{VERBOSE_REPORT;
  n =mat.n;
  delete [] array;
  array =new comple[n*n];
  delete [] darray;
  darray =new comple*[n];
  for(int i=0; i<n; i++){ darray[i] =&array[i*n]; }
  
  zcopy_(mat.n*mat.n, mat.array, 1, array, 1);
}

//=============================================================================
/*! make a shallow copy of the matrix\n
  This function is not designed to be used in project codes. */
inline void zhematrix::shallow_copy(const _zhematrix& mat)
{VERBOSE_REPORT;
  n =mat.n;
  delete [] array;
  array =mat.array;
  delete [] darray;
  darray =mat.darray;
  
  mat.nullify();
}

//=============================================================================
/*! resize the matrix */
inline void zhematrix::resize(const long& _n)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if( _n<0 ){
    ERROR_REPORT;
    std::cerr << "Matrix sizes must be positive integers." << std::endl
              << "Your input was (" << _n << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  n =_n;
  delete [] array;
  array =new comple[n*n];
  delete [] darray;
  darray =new comple*[n];
  for(int i=0; i<n; i++){ darray[i] =&array[i*n]; }
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! get row of the matrix */
inline _zrovector zhematrix::row(const long& _m) const
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if( _m<0 || _m>m ){
    ERROR_REPORT;
    std::cerr << "Input row number must be between 0 and " << m << "." << std::endl
              << "Your input was " << _m << "." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  zrovector v(n);
  for(long j=0; j<n; j++){ v(j)=(*this)(_m,j); }
  return _(v);
}

//=============================================================================
/*! get column of the matrix */
inline _zcovector zhematrix::col(const long& _n) const
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if( _n<0 || _n>n ){
    ERROR_REPORT;
    std::cerr << "Input row number must be between 0 and " << n << "." << std::endl
              << "Your input was " << _n << "." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  zcovector v(m);
  for(long i=0; i<m; i++){ v(i)=(*this)(i,_n); }
  return _(v);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! swap two matrices */
inline void swap(zhematrix& A, zhematrix& B)
{VERBOSE_REPORT;
  long A_n(A.n);
  comple* A_array(A.array);
  //comple** A_darray(A.darray);
  comple** A_darray=A.darray; //corruption to support VC++
  A.n=B.n; A.array=B.array; A.darray=B.darray;
  B.n=A_n; B.array=A_array; B.darray=A_darray;
}

//=============================================================================
/*! convert user object to smart-temporary object */
inline _zhematrix _(zhematrix& mat)
{VERBOSE_REPORT;
  _zhematrix newmat;
  
  //////// shallow copy ////////
  newmat.n =mat.n;
  newmat.array =mat.array;
  newmat.darray =mat.darray;
  
  //////// nullify ////////
  mat.n =0;
  mat.array =NULL;
  mat.darray =NULL;
  
  return newmat;
}
