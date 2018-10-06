//=============================================================================
/*! calculate vector product for 2D vector */
inline comple operator/(const zrovec2& A, const zrovec2& B)
{VERBOSE_REPORT;
  return A(0)*B(1)-A(1)*B(0);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! calculate vector product only for 3D vector */
inline zrovec3 operator/(const zrovec3& A, const zrovec3& B)
{VERBOSE_REPORT;
  zrovec3 C;
  C(0) =A(1)*B(2) -A(2)*B(1);
  C(1) =A(2)*B(0) -A(0)*B(2);
  C(2) =A(0)*B(1) -A(1)*B(0);
  return C;
}
