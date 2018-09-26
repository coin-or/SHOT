//=============================================================================
/*! calculate vector product only for 2D vector */
inline comple operator/(const zcovec2& A, const zcovec2& B)
{VERBOSE_REPORT;
  return A(0)*B(1) -A(1)*B(0);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! calculate vector product only for 3D vector */
inline zcovec3 operator/(const zcovec3& A, const zcovec3& B)
{VERBOSE_REPORT;
  zcovec3 C;
  C(0) =A(1)*B(2) -A(2)*B(1);
  C(1) =A(2)*B(0) -A(0)*B(2);
  C(2) =A(0)*B(1) -A(1)*B(0);
  return C;
}

//=============================================================================
/*! calculate vector product only for 3D vector */
inline zcovec3 operator/=(zcovec3& A, const zcovec3& B)
{VERBOSE_REPORT;
  A =A/B;
  return A;
}
