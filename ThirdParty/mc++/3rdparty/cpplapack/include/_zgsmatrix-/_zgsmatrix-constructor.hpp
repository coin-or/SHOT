//=============================================================================
/*! _zgsmatrix constructor without arguments */
inline _zgsmatrix::_zgsmatrix()
{VERBOSE_REPORT;
  //////// initialize ////////
  m =0;
  n =0;
  data.clear();
  rows.clear();
  cols.clear();
}

//=============================================================================
/*! _zgsmatrix copy constructor */
inline _zgsmatrix::_zgsmatrix(const _zgsmatrix& mat)
{VERBOSE_REPORT;
  //////// initialize ////////
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
/*! _zgsmatrix destructor */
inline _zgsmatrix::~_zgsmatrix()
{VERBOSE_REPORT;
  data.clear();
  rows.clear();
  cols.clear();
}
