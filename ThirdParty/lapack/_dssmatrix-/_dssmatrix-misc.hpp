//=============================================================================
/*! nullify all the matrix data */
inline void _dssmatrix::nullify() const
{VERBOSE_REPORT;
  n=0;
  data.clear();
  line.clear();
}

//=============================================================================
/*! destroy all the matrix data */
inline void _dssmatrix::destroy() const
{VERBOSE_REPORT;
  data.clear();
  line.clear();
}
