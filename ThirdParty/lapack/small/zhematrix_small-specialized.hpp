//=============================================================================
/*! calculate determinant */
inline comple det(const zhemat2& A)
{VERBOSE_REPORT;
  return A(0,0)*A(1,1) -A(1,0)*A(1,0);
}

//=============================================================================
/*! calculate inverse */
inline zhemat2 inv(const zhemat2& A)
{VERBOSE_REPORT;
  const comple Adet( det(A) );
  zhemat2 Ainv;
  Ainv(0,0)= A(1,1)/Adet;
  Ainv(1,0)=-A(1,0)/Adet;  Ainv(1,1)= A(0,0)/Adet;
  return Ainv;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! calculate determinant */
inline comple det(const zhemat3& A)
{VERBOSE_REPORT;
  return
    +A(0,0)*A(1,1)*A(2,2) -A(0,0)*A(2,1)*A(2,1)
    +A(1,0)*A(2,1)*A(2,0) -A(1,0)*A(1,0)*A(2,2)
    +A(2,0)*A(1,0)*A(2,1) -A(2,0)*A(1,1)*A(2,0);
}

//=============================================================================
/*! calculate inverse */
inline zhemat3 inv(const zhemat3& A)
{VERBOSE_REPORT;
  const comple Adet( det(A) );
  zhemat3 Ainv;
  Ainv(0,0) =(A(1,1)*A(2,2)-A(2,1)*A(2,1))/Adet;
  Ainv(1,0) =(A(2,1)*A(2,0)-A(1,0)*A(2,2))/Adet;
  Ainv(1,1) =(A(0,0)*A(2,2)-A(2,0)*A(2,0))/Adet;
  Ainv(2,0) =(A(1,0)*A(2,1)-A(1,1)*A(2,0))/Adet;
  Ainv(2,1) =(A(1,0)*A(2,0)-A(0,0)*A(2,1))/Adet;
  Ainv(2,2) =(A(0,0)*A(1,1)-A(1,0)*A(1,0))/Adet;
  return Ainv;
}
