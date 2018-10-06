//=============================================================================
/*! nullify all the matrix data */
inline void _zgsmatrix::nullify() const
{VERBOSE_REPORT;
  m=0;
  n=0;
  data.clear();
  rows.clear();
  cols.clear();
}

//=============================================================================
/*! destroy all the matrix data */
inline void _zgsmatrix::destroy() const
{VERBOSE_REPORT;
  data.clear();
  rows.clear();
  cols.clear();
}
