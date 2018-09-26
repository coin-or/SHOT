//=============================================================================
/*! calculate determinant */
inline double det(const dgemat2& A)
{VERBOSE_REPORT;
  return A(0,0)*A(1,1)-A(0,1)*A(1,0);
}

//=============================================================================
/*! calculate inverse */
inline dgemat2 inv(const dgemat2& A)
{VERBOSE_REPORT;
  const double Adet( det(A) );
  dgemat2 Ainv;
  Ainv(0,0)= A(1,1)/Adet;  Ainv(0,1)=-A(0,1)/Adet;
  Ainv(1,0)=-A(1,0)/Adet;  Ainv(1,1)= A(0,0)/Adet;
  return Ainv;
}

//=============================================================================
/*! return rotated tensor */
inline dgemat2 rotate(const dgemat2& m, const double& theta)
{VERBOSE_REPORT;
  //dgemat2 R(t2m(theta)); return R*m*t(R);//too slow
  double c(cos(theta)), s(sin(theta));
  double cc(c*c), cs(c*s), ss(s*s);
  dgemat2 mat;
  mat(0,0) =m(0,0)*cc -(m(0,1)+m(1,0))*cs +m(1,1)*ss;
  mat(0,1) =m(0,1)*cc +(m(0,0)-m(1,1))*cs -m(1,0)*ss;
  mat(1,0) =m(1,0)*cc +(m(0,0)-m(1,1))*cs -m(0,1)*ss;
  mat(1,1) =m(1,1)*cc +(m(0,1)+m(1,0))*cs +m(0,0)*ss;
  return mat;
}

//=============================================================================
/*! convert theta to 2x2 rotational matrix */
inline dgemat2 t2m(const double& theta)
{VERBOSE_REPORT;
  dgemat2 R;
  R(0,0)=cos(theta); R(0,1)=-sin(theta);
  R(1,0)=sin(theta); R(1,1)=cos(theta);
  return R;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! calculate determinant */
inline double det(const dgemat3& A)
{VERBOSE_REPORT;
  return
    +A(0,0)*A(1,1)*A(2,2) -A(0,0)*A(1,2)*A(2,1)
    +A(0,1)*A(1,2)*A(2,0) -A(0,1)*A(1,0)*A(2,2)
    +A(0,2)*A(1,0)*A(2,1) -A(0,2)*A(1,1)*A(2,0);
}

//=============================================================================
/*! calculate inverse */
inline dgemat3 inv(const dgemat3& A)
{VERBOSE_REPORT;
  const double Adet( det(A) );
  dgemat3 Ainv;
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

//=============================================================================
/*!  */
inline dgemat3 rotate(const dgemat3& m, const dquater& q)
{VERBOSE_REPORT;
  dgemat3 R(q2m(q));
  return R*m*t(R);
}

//=============================================================================
/*!  */
inline dquater m2q(const dgemat3& m)
{VERBOSE_REPORT;
  dcovec3 v( m(2,1)-m(1,2), m(0,2)-m(2,0), m(1,0)-m(0,1) );
  double t( acos(0.5*(m(0,0)+m(1,1)+m(2,2)-1.)) );
  return vt2q(v,t);
}
