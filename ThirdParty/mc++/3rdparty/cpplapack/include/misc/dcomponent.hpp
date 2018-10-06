//=============================================================================
//! Component Class for Real Double-precision Sparse Matrix Classes
class dcomponent
{
public:
  ///////////////////////////////////////////////
  /////////////////// objects ///////////////////
  ///////////////////////////////////////////////
  uint32_t i; //!< i index of the component
  uint32_t j; //!< j index of the component
  double v; //!< value of the component
  
  ///////////////////////////////////////////////
  ///////////////// constructors ////////////////
  ///////////////////////////////////////////////
  inline dcomponent(){ ; }
  inline dcomponent(const uint32_t& _i, const uint32_t& _j, const double& _v) :i(_i), j(_j), v(_v){ ; }
  
  ///////////////////////////////////////////////
  ////////////////// functions //////////////////
  ///////////////////////////////////////////////
  inline friend std::ostream& operator<<(std::ostream&, const dcomponent&);
  inline static bool ilt(const dcomponent&, const dcomponent&);
  inline static bool jlt(const dcomponent&, const dcomponent&);
};

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
inline std::ostream& operator<<(std::ostream& s, const dcomponent& c)
{VERBOSE_REPORT;
  s << "(" << c.i << ", " << c.j << ",  " << c.v << ")" << std::flush;
  return s;
}

//=============================================================================
/*! lessthan function for i of dcomponent */
inline bool dcomponent::ilt(const dcomponent& a, const dcomponent& b)
{VERBOSE_REPORT;
  return a.i < b.i;
}

//=============================================================================
/*! lessthan function for j of dcomponent */
inline bool dcomponent::jlt(const dcomponent& a, const dcomponent& b)
{VERBOSE_REPORT;
  return a.j < b.j;
}
