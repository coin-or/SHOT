//=============================================================================
/*! +zgbmatrix operator */
inline const zgbmatrix& operator+(const zgbmatrix& mat)
{VERBOSE_REPORT;
  return mat;
}

//=============================================================================
/*! -zgbmatrix operator */
inline _zgbmatrix operator-(const zgbmatrix& mat)
{VERBOSE_REPORT;
  zgbmatrix newmat(mat.m,mat.n,mat.kl,mat.ku);
  for(long i=0; i<(newmat.kl+newmat.ku+1)*newmat.n; i++){
    newmat.array[i]=-mat.array[i];
  }
  
  return _(newmat);
}
