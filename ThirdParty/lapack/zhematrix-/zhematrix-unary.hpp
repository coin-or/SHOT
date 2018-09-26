//=============================================================================
/*! +zhematrix operator */
inline const zhematrix& operator+(const zhematrix& mat)
{VERBOSE_REPORT;
  return mat;
}

//=============================================================================
/*! -zgematrix operator */
inline _zhematrix operator-(const zhematrix& mat)
{VERBOSE_REPORT;
  zhematrix newmat(mat.n);
  for(long i=0; i<mat.n; i++){ for(long j=0; j<=i; j++){
    newmat(i,j) =-mat(i,j);
  }}
  
  return _(newmat);
}
