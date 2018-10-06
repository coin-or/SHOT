//=============================================================================
/*! _dgsmatrix constructor without arguments */
inline _dgsmatrix::_dgsmatrix()
{VERBOSE_REPORT;
  m =0;
  n =0;
  data.clear();
  rows.clear();
  cols.clear();
}

//=============================================================================
/*! _dgsmatrix copy constructor */
inline _dgsmatrix::_dgsmatrix(const _dgsmatrix& mat)
{VERBOSE_REPORT;
  m =mat.m;
  n =mat.n;
  data.swap(mat.data);
  rows.swap(mat.rows);
  cols.swap(mat.cols);
  
  mat.nullify();
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! _dgsmatrix destructor */
inline _dgsmatrix::~_dgsmatrix()
{VERBOSE_REPORT;
  data.clear();
  rows.clear();
  cols.clear();
}
