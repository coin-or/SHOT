//=============================================================================
/*! calculate determinant */
inline comple det(const zgemat2& A)
{VERBOSE_REPORT;
  return A(0,0)*A(1,1)-A(0,1)*A(1,0);
}

//=============================================================================
/*! calculate inverse */
inline zgemat2 inv(const zgemat2& A)
{VERBOSE_REPORT;
  const comple Adet( det(A) );
  zgemat2 Ainv;
  Ainv(0,0)= A(1,1)/Adet;  Ainv(0,1)=-A(0,1)/Adet;
  Ainv(1,0)=-A(1,0)/Adet;  Ainv(1,1)= A(0,0)/Adet;
  return Ainv;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! calculate determinant */
inline comple det(const zgemat3& A)
{VERBOSE_REPORT;
  return
    +A(0,0)*A(1,1)*A(2,2) -A(0,0)*A(1,2)*A(2,1)
    +A(0,1)*A(1,2)*A(2,0) -A(0,1)*A(1,0)*A(2,2)
    +A(0,2)*A(1,0)*A(2,1) -A(0,2)*A(1,1)*A(2,0);
}

//=============================================================================
/*! calculate inverse */
inline zgemat3 inv(const zgemat3& A)
{VERBOSE_REPORT;
  const comple Adet( det(A) );
  zgemat3 Ainv;
  Ainv(0,0) =(A(1,1)*A(2,2)-A(1,2)*A(2,1))/Adet;
  Ainv(0,1) =(A(0,2)*A(2,1)-A(0,1)*A(2,2))/Adet;
  Ainv(0,2) =(A(0,1)*A(1,2)-A(0,2)*A(1,1))/Adet;
  Ainv(1,0) =(A(1,2)*A(2,0)-A(1,0)*A(2,2))/Adet;
  Ainv(1,1) =(A(0,0)*A(2,2)-A(0,2)*A(2,0))/Adet;
  Ainv(1,2) =(A(0,2)*A(1,0)-A(0,0)*A(1,2))/Adet;
  Ainv(2,0) =(A(1,0)*A(2,1)-A(1,1)*A(2,0))/Adet;
  Ainv(2,1) =(A(0,1)*A(2,0)-A(0,0)*A(2,1))/Adet;
  Ainv(2,2) =(A(0,0)*A(1,1)-A(0,1)*A(1,0))/Adet;
  return Ainv;
}
