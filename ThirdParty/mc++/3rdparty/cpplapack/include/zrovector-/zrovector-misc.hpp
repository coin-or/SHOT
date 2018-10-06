//=============================================================================
/*! clear vector */
inline void zrovector::clear()
{VERBOSE_REPORT;
  l =0;
  delete [] array;
  array =NULL;
}

//=============================================================================
/*! make vector into zero vector */
inline zrovector& zrovector::zero()
{VERBOSE_REPORT;
  for(long i=0; i<l; i++){ array[i] =comple(0.0,0.0); }
  return *this;
}

//=============================================================================
/*! change sign(+/-) of the vector */
inline void zrovector::chsign()
{VERBOSE_REPORT;
  for(long i=0; i<l; i++){ array[i] =-array[i]; }
}

//=============================================================================
/*! make a deep copy of the zrovector */
inline void zrovector::copy(const zrovector& vec)
{VERBOSE_REPORT;
  delete [] array;
  l =vec.l;
  array =new comple[vec.l];
  zcopy_(vec.l, vec.array, 1, array, 1);
}

//=============================================================================
/*! make a shallow copy of the vector\n
 This function is not desinged to be used in project codes. */
inline void zrovector::shallow_copy(const _zrovector& vec)
{VERBOSE_REPORT;
  l =vec.l;
  delete [] array;
  array =vec.array;
  
  vec.nullify();
}

//=============================================================================
/*! make an alias of the vector\n
  Be carefull to use this function not to cause double free. */
inline void zrovector::alias(const zrovector& vec)
{VERBOSE_REPORT;
  l =vec.l;
  //cap =vec.cap;
  delete [] array;
  array =vec.array;
}

//=============================================================================
/*! unalias the vector */
inline void zrovector::unalias()
{VERBOSE_REPORT;
  l =0;
  //cap =0;
  array =NULL;
}

//=============================================================================
/*! resize vector */
inline void zrovector::resize(const long& _l)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if( _l<0 ){
    ERROR_REPORT;
    std::cerr << "Vector size must be positive integers." << std::endl
              << "Your input was (" << _l << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  l =_l;
  delete [] array;
  array =new comple[_l];
}

//=============================================================================
/*! swap two vectors */
inline void swap(zrovector& u, zrovector& v)
{VERBOSE_REPORT;
  long u_L(u.l);
  comple* u_Array(u.array);
  u.l=v.l; u.array=v.array;
  v.l=u_L; v.array=u_Array;
}

//=============================================================================
/*! convert user object to smart-temporary object */
inline _zrovector _(zrovector& vec)
{VERBOSE_REPORT;
  _zrovector newvec;
  
  //////// shallow copy ////////
  newvec.l =vec.l;
  newvec.array =vec.array;
  
  //////// nullify ////////
  vec.l =0;
  vec.array =NULL;
  
  return newvec;
}
