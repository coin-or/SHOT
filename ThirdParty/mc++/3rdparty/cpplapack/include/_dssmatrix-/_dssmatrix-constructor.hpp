//=============================================================================
/*! _dssmatrix constructor without arguments */
inline _dssmatrix::_dssmatrix()
  :m(n)
{VERBOSE_REPORT;
  n =0;
  data.clear();
  line.clear();
}

//=============================================================================
/*! _dssmatrix copy constructor */
inline _dssmatrix::_dssmatrix(const _dssmatrix& mat)
  :m(n)
{VERBOSE_REPORT;
  n =mat.n;
  data.swap(mat.data);
  line.swap(mat.line);
  
  mat.nullify();
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! _dssmatrix destructor */
inline _dssmatrix::~_dssmatrix()
{VERBOSE_REPORT;
  data.clear();
  line.clear();
}
