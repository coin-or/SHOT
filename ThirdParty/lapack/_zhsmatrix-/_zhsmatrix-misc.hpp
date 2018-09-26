//=============================================================================
/*! nullify all the matrix data */
inline void _zhsmatrix::nullify() const
{VERBOSE_REPORT;
  n=0;
  data.clear();
  line.clear();
}

//=============================================================================
/*! destroy all the matrix data */
inline void _zhsmatrix::destroy() const
{VERBOSE_REPORT;
  data.clear();
  line.clear();
}
