//=============================================================================
/*! nullify all the matrix data */
inline void _zhematrix::nullify() const
{VERBOSE_REPORT;
  n=0;
  array=NULL;
  darray=NULL;
}

//=============================================================================
/*! destroy all the matrix data */
inline void _zhematrix::destroy() const
{VERBOSE_REPORT;
  delete [] array;
  delete [] darray;
  array=NULL;
  darray=NULL;
}

//=============================================================================
/*! complete the upper-right components */
inline void _zhematrix::complete() const
{VERBOSE_REPORT;
  for(long i=0; i<n; i++){
    for(long j=0; j<i; j++){
      darray[i][j] =std::conj(darray[j][i]);
    }
  }
}

