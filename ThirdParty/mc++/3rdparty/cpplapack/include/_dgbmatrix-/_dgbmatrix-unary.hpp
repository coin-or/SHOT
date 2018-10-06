//=============================================================================
/*! +_dgbmatrix operator */
inline const _dgbmatrix& operator+(const _dgbmatrix& mat)
{VERBOSE_REPORT;
  return mat;
}

//=============================================================================
/*! -_dgbmatrix operator */
inline _dgbmatrix operator-(const _dgbmatrix& mat)
{VERBOSE_REPORT;
  for(long i=0; i<(mat.kl+mat.ku+1)*mat.n; i++){
    mat.array[i]=-mat.array[i];
  }
  
  return mat;
}
