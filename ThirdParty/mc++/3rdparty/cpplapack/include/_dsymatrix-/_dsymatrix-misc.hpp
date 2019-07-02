//=============================================================================
/*! nullify all the matrix data */
inline void _dsymatrix::nullify() const
{VERBOSE_REPORT;
  n=0;
  array=NULL;
  darray=NULL;
}

//=============================================================================
/*! destroy all the matrix data */
inline void _dsymatrix::destroy() const
{VERBOSE_REPORT;
  delete [] array;
  delete [] darray;
  array=NULL;
  darray=NULL;
}

//=============================================================================
/*! complete the upper-right components */
inline void _dsymatrix::complete() const
{VERBOSE_REPORT;
  for(long i=0; i<n; i++){
    for(long j=0; j<i; j++){
      darray[i][j] =darray[j][i];
    }
  }
}
