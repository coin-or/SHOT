//=============================================================================
/*! +_zhematrix operator */
inline const _zhematrix& operator+(const _zhematrix& mat)
{VERBOSE_REPORT;
  return mat;
}

//=============================================================================
/*! -_zhematrix operator */
inline _zhematrix operator-(const _zhematrix& mat)
{VERBOSE_REPORT;
  for(long i=0; i<mat.n; i++){
    for(long j=0; j<=i; j++){
      mat.array[i+mat.n*j] =-mat.array[i+mat.n*j];
    }
  }
  
  return mat;
}
