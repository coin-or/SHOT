//=============================================================================
/*! clear vector */
inline void drovector::clear()
{VERBOSE_REPORT;
  l =0;
  cap =0;
  delete [] array;
  array =NULL;
}

//=============================================================================
/*! make vector into zero vector */
inline drovector& drovector::zero()
{VERBOSE_REPORT;
  for(long i=0; i<l; i++){ array[i] =0.0; }
  return *this;
}

//=============================================================================
/*! change sign(+/-) of the vector */
inline void drovector::chsign()
{VERBOSE_REPORT;
  for(long i=0; i<l; i++){ array[i] =-array[i]; }
}

//=============================================================================
/*! make a deep copy of the drovector */
inline void drovector::copy(const drovector& vec)
{VERBOSE_REPORT;
  l =vec.l;
  cap =vec.cap;
  delete [] array;
  array =new double[vec.cap];
  dcopy_(vec.l, vec.array, 1, array, 1);
}

//=============================================================================
/*! make a shallow copy of the vector\n
 This function is not desinged to be used in project codes. */
inline void drovector::shallow_copy(const _drovector& vec)
{VERBOSE_REPORT;
  l =vec.l;
  cap =vec.cap;
  delete [] array;
  array =vec.array;
  
  vec.nullify();
}

//=============================================================================
/*! make an alias of the vector\n
  Be carefull to use this function not to cause double free. */
inline void drovector::alias(const drovector& vec)
{VERBOSE_REPORT;
  l =vec.l;
  cap =vec.cap;
  delete [] array;
  array =vec.array;
}

//=============================================================================
/*! unalias the vector */
inline void drovector::unalias()
{VERBOSE_REPORT;
  l =0;
  cap =0;
  array =NULL;
}

//=============================================================================
/*! resize vector */
inline drovector& drovector::resize(const long& _l, const long margin)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if( _l<0 || margin<0 ){
    ERROR_REPORT;
    std::cerr << "Vector size must be positive integers." << std::endl
              << "Your input was (" << _l << ", " << margin << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  l =_l;
  cap =l+margin;
  delete [] array;
  array =new double[cap];
  
  return *this;
}

//=============================================================================
/*! stretch or shrink vector */
inline void drovector::stretch(const long& dl)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if( l+dl<0 ){
    ERROR_REPORT;
    std::cerr << "Vector size must be positive integers." << std::endl
              << "Your input was (" << dl << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  //////// zero ////////
  if(dl==0){ return; }
  
  //////// non-zero ////////
  l +=dl;

  if(l>cap){
    while(l>cap){
      cap++;
      cap*=2;
    }
    double* newArray(new double[cap]);
    dcopy_(l-dl, array, 1, newArray, 1);
    delete [] array;
    array =newArray;
  }
}

//=============================================================================
/*! swap two vectors */
inline void swap(drovector& u, drovector& v)
{VERBOSE_REPORT;
  long u_cap(u.cap), u_l(u.l);
  double* u_array(u.array);
  u.l=v.l; u.cap=v.cap; u.array=v.array;
  v.l=u_l; v.cap=u_cap; v.array=u_array;
}

//=============================================================================
/*! convert user object to smart-temporary object */
inline _drovector _(drovector& vec)
{VERBOSE_REPORT;
  _drovector newvec;
  
  //////// shallow copy ////////
  newvec.l =vec.l;
  newvec.cap =vec.cap;
  newvec.array =vec.array;
  
  //////// nullify ////////
  vec.l =0;
  vec.cap =0;
  vec.array =NULL;
  
  return newvec;
}
