//=============================================================================
/*! +zgematrix operator */
inline const zgematrix& operator+(const zgematrix& mat)
{VERBOSE_REPORT;
  return mat;
}

//=============================================================================
/*! -zgematrix operator */
inline _zgematrix operator-(const zgematrix& mat)
{VERBOSE_REPORT;
  zgematrix newmat(mat.m,mat.n);
  for(long i=0; i<newmat.m*newmat.n; i++){ newmat.array[i]=-mat.array[i]; }
  
  return _(newmat);
}
