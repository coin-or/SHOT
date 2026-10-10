// Copyright (C) Benoit Chachuat, Imperial College London.
// All Rights Reserved.
// This code is published under the EPL-2.0 with GPL-2.0-or-later as a Secondary License; see the LICENSE file.
// SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later

/*!
\page page_INTERVAL Non-Verified Interval Arithmetic for Factorable Functions
\author Beno&icirc;t Chachuat

Computational methods for enclosing the range of functions find their origins in
interval analysis back in the early 1960s [Moore, 1966; Moore <I>et al.</I>,
2009]. For functions whose expressions can be broken down into a finite number
of elementary unary and binary operations, namely factorable functions, interval
bounding can be readily automated. The class mc::Interval provides a basic
implementation of interval arithmetic, which is <B>not a verified
implementation</B> in the sense that rounding errors are not accounted for. For
verified interval computations, it is strongly recommended to use third-party
libraries such as <A
href="http://www.ti3.tu-harburg.de/Software/PROFILEnglisch.html">PROFIL</A> or
<A href="http://www.math.uni-wuppertal.de/~xsc/software/filib.html">FILIB++</A>.

The implementation of mc::Interval relies on the operator/function overloading
mechanism of C++. This makes the computation of bounds both simple and
intuitive, similar to computing function values in real arithmetics. Moreover,
mc::Interval can be used as the underlying interval type in other classes of
MC++ via templates; e.g., mc::McCormick<mc::Interval>, mc::TModel<mc::Interval>,
mc::TVar<mc::Interval>.


\section sec_INTERVAL_use How do I compute interval bounds on the range of a
factorable function?

Suppose we want to calculate bounds on the range of the real-valued function
\f$f(x,y)=x(\exp(x)-y)^2\f$ for \f$(x,y)\in [-2,1]^2\f$.

First, we shall define the variables \f$x\f$ and \f$y\f$. This is done as
follows:

\code
      #include "interval.hpp"
      typedef mc::Interval I;

      I X( -2., 1. );
      I Y( -2., 1. );
\endcode

Essentially, the last two lines mean that <tt>X</tt> and <tt>Y</tt> are variable
of type mc::Interval, both defined as \f$[-2,1]\f$.

Having defined the variables, bounds on the range of \f$f\f$ on \f$[-2,1]^2\f$
are simply calculated as

\code
      I F = X*pow(exp(X)-Y,2);
\endcode

These bounds can be displayed to the standard output as:

\code
      std::cout << "F bounds: " << F << std::endl;
\endcode

which produces the following output:

\verbatim
F bounds: [ -4.45244e+01 :  2.22622e+01 ]
\endverbatim

Moreover, the upper and lower bounds in the interval bound <a>F</a> can be
retrieved as:

\code
      double Fl = IF.l();
      double Fu = IF.u();
\endcode <tt>exp</tt>,


\section sec_INTERVAL_fct Which functions are overloaded in mc::Interval?

mc::Interval overloads the usual functions <tt>exp</tt>, <tt>log</tt>,
<tt>sqr</tt>, <tt>sqrt</tt>, <tt>pow</tt>, <tt>inv</tt>, <tt>cos</tt>,
<tt>sin</tt>, <tt>tan</tt>, <tt>acos</tt>, <tt>asin</tt>, <tt>atan</tt>,
<tt>erf</tt>, <tt>erfc</tt>, <tt>min</tt>, <tt>max</tt>, <tt>fabs</tt>.
mc::Interval also defines the following functions:
- <tt>inter(Z,X,Y)</tt>, computing the intersection \f$Z = X\cap Y\f$ and
returning true/false if the intersection is nonempty/empty
- <tt>hull(X,Y)</tt>, returning the interval hull of \f$X\cup Y\f$
- <tt>diam(X)</tt>, returning the diameter of \f$X\f$
- <tt>mid(X)</tt>, returning the mid-point of \f$X\f$
- <tt>abs(X)</tt>, returning the absolute value of \f$X\f$
.


\section sec_INTERVAL_opt What are the options in mc::Interval and how are they
set?

The class mc::Interval has a public static member called mc::Interval::options
that can be used to set/modify the options; e.g.,

\code
      mc::Interval::options.DISPLAY_DIGITS = 7;
\endcode

The available options are as follows:

<TABLE border="1">
<CAPTION><EM>Options in mc::Interval::Options: name, type and
description</EM></CAPTION> <TR><TH><b>Name</b> <TD><b>Type</b><TD><b>Default</b>
         <TD><b>Description</b>
     <TR><TH><tt>DISPLAY_DIGITS</tt> <TD><tt>unsigned int</tt> <TD>5
         <TD>Number of digits in output stream
</TABLE>


\section sec_INTERVAL_err What errors can be encountered in using mc::Interval?

Errors are managed based on the exception handling mechanism of the C++
language. Each time an error is encountered, a class object of type
Interval::Exceptions is thrown, which contains the type of error.

Possible errors encountered in using mc::Interval are:

<TABLE border="1">
<CAPTION><EM>Exceptions in mc::Interval</EM></CAPTION>
     <TR><TH><b>Number</b> <TD><b>Description</b>
     <TR><TH><tt>1</tt> <TD>Division by zero
     <TR><TH><tt>2</tt> <TD>Inverse with zero in range
     <TR><TH><tt>3</tt> <TD>Log with negative values in range
     <TR><TH><tt>4</tt> <TD>Square-root with nonpositive values in range
     <TR><TH><tt>5</tt> <TD>Inverse cosine with values outside of [-1,1] range
     <TR><TH><tt>6</tt> <TD>Inverse sine with values outside of [-1,1] range
     <TR><TH><tt>7</tt> <TD>Tangent with values \f$\frac{\pi}{2}+k\,\pi\f$, with
\f$k\in\mathbb{Z}\f$, in range
</TABLE>

\section sec_INTERVAL_refs References

- Moore, R.E., <I><A
href="http://books.google.co.uk/books/about/Interval_analysis.html?id=csQ-AAAAIAAJ&redir_esc=y2">"Interval
Analysis"</A></I>, Prentice-Hall, 1966
- Moore, R.E., M.J. Cloud, R.B. Kearfott, <I><A
href="http://books.google.co.uk/books/about/Introduction_to_interval_analysis.html?id=tT7ykKbqfEwC&redir_esc=y">"Introduction
to Interval Analysis"</A></I>, SIAM, 2009
.

*/

#ifndef MC__INTERVAL_HPP
#define MC__INTERVAL_HPP

#include <stdarg.h>

#include <iomanip>
#include <iostream>

#include "mcfunc.hpp"

// #define MC__INTERVAL_TRACE

namespace mc
{
//! @brief C++ class for (non-verified) interval bounding of factorable function
////////////////////////////////////////////////////////////////////////
//! mc::Interval is a C++ class for interval bounding of factorable
//! functions on a box based on natural interval extensions. Round-off
//! errors are not accounted for in the computations (non-verified
//! implementation).
////////////////////////////////////////////////////////////////////////
class Interval
////////////////////////////////////////////////////////////////////////
{
  // friends of class Interval for operator overloading
  friend Interval operator+(Interval const&);
  friend Interval operator+(Interval const&, Interval const&);
  friend Interval&& operator+(Interval&&, Interval const&) noexcept;
  friend Interval operator+(double const&, Interval const&);
  friend Interval&& operator+(double const&, Interval&&) noexcept;
  friend Interval operator+(Interval const&, double const&);
  friend Interval&& operator+(Interval&&, double const&) noexcept;
  friend Interval operator-(Interval const&);
  friend Interval operator-(Interval const&, Interval const&);
  friend Interval&& operator-(Interval&&, Interval const&) noexcept;
  friend Interval operator-(double const&, Interval const&);
  friend Interval operator-(Interval const&, double const&);
  friend Interval&& operator-(Interval&&, double const&) noexcept;
  friend Interval operator*(Interval const&, Interval const&);
  friend Interval operator*(Interval const&, double const&);
  friend Interval operator*(double const&, Interval const&);
  friend Interval operator/(Interval const&, Interval const&);
  friend Interval operator/(Interval const&, double const&);
  friend Interval operator/(double const&, Interval const&);
  friend std::ostream& operator<<(std::ostream&, Interval const&);
  friend bool operator==(Interval const&, Interval const&);
  friend bool operator!=(Interval const&, Interval const&);
  friend bool operator<=(Interval const&, Interval const&);
  friend bool operator>=(Interval const&, Interval const&);
  friend bool operator<(Interval const&, Interval const&);
  friend bool operator>(Interval const&, Interval const&);

  // friends of class Interval for function overloading
  friend double diam(Interval const&);
  friend double abs(Interval const&);
  friend double mid(Interval const&);
  friend Interval inv(Interval const&);
  friend Interval sqr(Interval const&);
  friend Interval exp(Interval const&);
  friend Interval&& exp(Interval&&);
  friend Interval log(Interval const&);
  friend Interval cos(Interval const&);
  friend Interval sin(Interval const&);
  friend Interval tan(Interval const&);
  friend Interval acos(Interval const&);
  friend Interval asin(Interval const&);
  friend Interval atan(Interval const&);
  friend Interval cosh(Interval const&);
  friend Interval sinh(Interval const&);
  friend Interval tanh(Interval const&);
  friend Interval fabs(Interval const&);
  friend Interval relu(Interval const&);
  friend Interval sqrt(Interval const&);
  friend Interval xlog(Interval const&);
  friend Interval lmtd(Interval const&, Interval const&);
  friend Interval rlmtd(Interval const&, Interval const&);
  friend Interval erf(Interval const&);
  friend Interval erfc(Interval const&);
  friend Interval fstep(Interval const&);
  friend Interval bstep(Interval const&);
  friend Interval arrh(Interval const&, double const&);
  friend Interval pow(Interval const&, const int);
  friend Interval pow(Interval const&, double const&);
  friend Interval pow(Interval const&, Interval const&);
  friend Interval prod(const unsigned int, const Interval*);
  friend Interval monom(const unsigned int, const Interval*, const unsigned*);
  friend Interval cheb(Interval const&, const unsigned);
  friend Interval hull(Interval const&, Interval const&);
  friend Interval min(Interval const&, Interval const&);
  friend Interval max(Interval const&, Interval const&);
  friend Interval min(const unsigned int, const Interval*);
  friend Interval max(const unsigned int, const Interval*);
  friend bool inter(Interval&, Interval const&, Interval const&);

 public:
  // other operator overloadings (inline)
  Interval&
  operator=(double const& c)
  {
    _l = c;
    _u = c;
    return *this;
  }
  Interval&
  operator=(Interval const& I)
  {
#ifdef MC__INTERVAL_TRACE
    std::cout << "In operator=(Interval const&)\n";
#endif
    if (this != &I)
    {
      _l = I._l;
      _u = I._u;
    }
    return *this;
  }
  Interval&
  operator=(Interval&& I) noexcept
  {
#ifdef MC__INTERVAL_TRACE
    std::cout << "In operator=(Interval &&)\n";
#endif
    if (this != &I)
    {
      _l = std::move(I._l);
      _u = std::move(I._u);
    }
    return *this;
  }
  Interval&
  operator+=(double const& c)
  {
    _l += c;
    _u += c;
    return *this;
  }
  Interval&
  operator+=(const Interval& I)
  {
    _l += I._l;
    _u += I._u;
    return *this;
  }
  Interval&
  operator-=(double const& c)
  {
    _l -= c;
    _u -= c;
    return *this;
  }
  Interval&
  operator-=(const Interval& I)
  {
    _l -= I._u;
    _u -= I._l;
    return *this;
  }
  Interval&
  operator*=(double const& c)
  {
    _l *= c;
    _u *= c;
    if (c < 0.) std::swap(_l, _u);
    return *this;
  }
  Interval&
  operator*=(Interval const& I)
  {
    double tmp = std::min(std::min(_l * I._l, _l * I._u),
                          std::min(_u * I._l, _u * I._u));
    _u         = std::max(std::max(_l * I._l, _l * I._u),
                          std::max(_u * I._l, _u * I._u));
    std::swap(_l, tmp);
    return *this;
  }
  Interval&
  operator/=(double const& c)
  {
    _l /= c;
    _u /= c;
    if (c < 0.) std::swap(_l, _u);
    return *this;
  }
  Interval&
  operator/=(Interval const& I)
  {
    return operator*=(inv(I));
  }

  /** @defgroup INTERVAL Non-Validated Interval Arithmetic for Factorable
   * Functions
   *  @{
   */
  //! @brief Options of mc::Interval
  static struct Options
  {
    //! @brief Constructor
    Options() : DISPLAY_DIGITS(5) {}
    //! @brief Copy of mc::Interval::Options
    Options(Options const& opt) { DISPLAY_DIGITS = opt.DISPLAY_DIGITS; }
    //! @brief Assignment of mc::Interval::Options
    Options&
    operator=(Options const& opt)
    {
      DISPLAY_DIGITS = opt.DISPLAY_DIGITS;
      return *this;
    }
    //! @brief Number of digits displayed with << operator (default=5)
    unsigned int DISPLAY_DIGITS;
  } options;

  //! @brief Exceptions of mc::Interval
  class Exceptions
  {
   public:
    //! @brief Enumeration type for mc::Interval exceptions
    enum TYPE
    {
      DIV = 1,  //!< Division by zero
      INV,      //!< Inverse with zero in range
      LOG,      //!< Log with negative values in range
      SQRT,     //!< Square-root with nonpositive values in range
      ACOS,     //!< Inverse cosine with values outside of [-1,1] range
      ASIN,     //!< Inverse sine with values outside of [-1,1] range
      TAN,      //!< Tangent with values \f$\frac{\pi}{2}+k\,\pi\f$, with
                //!< \f$k\in\mathbb{Z}\f$, in range
      CHEB      //!< Chebyshev basis function outside of [-1,1] range
    };
    //! @brief Constructor for error flag <a>ierr</a>
    Exceptions(TYPE ierr) : _ierr(ierr) {}
    //! @brief Return error flag
    int
    ierr()
    {
      return _ierr;
    }
    //! @brief Return error description
    std::string
    what()
    {
      switch (_ierr)
      {
        case DIV:
          return "mc::Interval\t Division by zero";
        case INV:
          return "mc::Interval\t Inverse with zero in range";
        case LOG:
          return "mc::Interval\t Log with negative values in range";
        case SQRT:
          return "mc::Interval\t Square-root with nonpositive values in range";
        case ACOS:
          return "mc::Interval\t Inverse cosine with values outside of [-1,1] "
                 "range";
        case ASIN:
          return "mc::Interval\t Inverse sine with values outside of [-1,1] "
                 "range";
        case TAN:
          return "mc::Interval\t Tangent with values pi/2+k*pi in range";
        case CHEB:
          return "mc::Interval\t Chebyshev basis outside of [-1,1] range";
      }
      return "mc::Interval\t Undocumented error";
    }

   private:
    TYPE _ierr;
  };
  //! @brief Default constructor (needed for arrays of mc::Interval elements)
  Interval() {}
  //! @brief Constructor for a constant value <a>c</a>
  Interval(double const& c) : _l(c), _u(c) {}
  //! @brief Constructor for a variable that belongs to the interval
  //! [<a>l</a>,<a>u</a>]
  Interval(double const& l, double const& u)
      : _l(l < u ? l : u), _u(l < u ? u : l)
  {
  }
  //! @brief Copy constructor for the interval <a>I</a>
  Interval(Interval const& I) : _l(I._l), _u(I._u)
  {
#ifdef MC__INTERVAL_TRACE
    std::cout << "In Interval(Interval const&)\n";
#endif
  }
  //! @brief Move constructor for the interval <a>I</a>
  Interval(Interval&& I) noexcept : _l(I._l), _u(I._u)
  {
#ifdef MC__INTERVAL_TRACE
    std::cout << "In Interval(Interval &&)\n";
#endif
  }
  //! @brief Destructor
  ~Interval() {}

  //! @brief Return lower bound
  double
  l() const
  {
    return _l;
  }
  //! @brief Return upper bound
  double
  u() const
  {
    return _u;
  }

  //! @brief Return/set lower bound to <a>lb</a>
  double&
  l()
  {
    return _l;
  }
  //! @brief Return/set upper bound to <a>ub</a>
  double&
  u()
  {
    return _u;
  }
  /** @} */

 private:
  //! @brief Lower bound
  double _l;
  //! @brief Upper bound
  double _u;
};

////////////////////////////////////////////////////////////////////////

inline Interval::Options Interval::options;

inline Interval
operator+(Interval const& I)
{
  return I;
}

inline Interval
operator-(Interval const& I)
{
  return Interval(-I._u, -I._l);
}

inline Interval
operator+(double const& c, Interval const& I)
{
#ifdef MC__INTERVAL_TRACE
  std::cout << "In operator+(double const&, Interval const&)\n";
#endif
  return Interval(c + I._l, c + I._u);
}

inline Interval&&
operator+(double const& c, Interval&& I) noexcept
{
#ifdef MC__INTERVAL_TRACE
  std::cout << "In operator+(double const&, Interval &&)\n";
#endif
  I._l += c;
  I._u += c;
  return std::move(I);
}

inline Interval
operator+(Interval const& I, double const& c)
{
#ifdef MC__INTERVAL_TRACE
  std::cout << "In operator+(Interval const&, double const&)\n";
#endif
  return Interval(c + I._l, c + I._u);
}

inline Interval&&
operator+(Interval&& I, double const& c) noexcept
{
#ifdef MC__INTERVAL_TRACE
  std::cout << "In operator+(Interval &&, double const&)\n";
#endif
  I._l += c;
  I._u += c;
  return std::move(I);
}

inline Interval
operator+(Interval const& I1, Interval const& I2)
{
#ifdef MC__INTERVAL_TRACE
  std::cout << "In operator+(Interval const&, Interval const&)\n";
#endif
  return Interval(I1._l + I2._l, I1._u + I2._u);
}

inline Interval&&
operator+(Interval&& I1, Interval const& I2) noexcept
{
#ifdef MC__INTERVAL_TRACE
  std::cout << "In operator+(Interval &&, Interval const&)\n";
#endif
  I1._l += I2._l;
  I1._u += I2._u;
  return std::move(I1);
}

inline Interval
operator-(double const& c, Interval const& I)
{
  return Interval(c - I._u, c - I._l);
}

inline Interval
operator-(Interval const& I, double const& c)
{
#ifdef MC__INTERVAL_TRACE
  std::cout << "In operator-(Interval &&, double const&)\n";
#endif
  return Interval(I._l - c, I._u - c);
}

inline Interval&&
operator-(Interval&& I, double const& c) noexcept
{
#ifdef MC__INTERVAL_TRACE
  std::cout << "In operator-(Interval &&, double const&)\n";
#endif
  I._l -= c;
  I._u -= c;
  return std::move(I);
}

inline Interval
operator-(Interval const& I1, Interval const& I2)
{
#ifdef MC__INTERVAL_TRACE
  std::cout << "In operator-(Interval const&, Interval const&)\n";
#endif
  return Interval(I1._l - I2._u, I1._u - I2._l);
}

inline Interval&&
operator-(Interval&& I1, Interval const& I2) noexcept
{
#ifdef MC__INTERVAL_TRACE
  std::cout << "In operator-(Interval &&, Interval const&)\n";
#endif
  I1._l -= I2._u;
  I1._u -= I2._l;
  return std::move(I1);
}

inline Interval
operator*(double const& c, Interval const& I)
{
  return Interval(c >= 0 ? c * I._l : c * I._u, c >= 0 ? c * I._u : c * I._l);
}

inline Interval
operator*(Interval const& I, double const& c)
{
  return Interval(c >= 0 ? c * I._l : c * I._u, c >= 0 ? c * I._u : c * I._l);
}

inline Interval
operator*(Interval const& I1, Interval const& I2)
{
  return Interval(std::min(std::min(I1._l * I2._l, I1._l * I2._u),
                           std::min(I1._u * I2._l, I1._u * I2._u)),
                  std::max(std::max(I1._l * I2._l, I1._l * I2._u),
                           std::max(I1._u * I2._l, I1._u * I2._u)));
}

inline Interval
operator/(Interval const& I, double const& c)
{
  if (isequal(c, 0.)) throw Interval::Exceptions(Interval::Exceptions::DIV);
  return (1. / c) * I;
}

inline Interval
operator/(double const& c, Interval const& I)
{
  return c * inv(I);
}

inline Interval
operator/(Interval const& I1, Interval const& I2)
{
  return I1 * inv(I2);
}

inline double
diam(Interval const& I)
{
  return I._u - I._l;
}

inline double
mid(Interval const& I)
{
  return 0.5 * (I._u + I._l);
}

inline double
abs(Interval const& I)
{
  return std::max(std::fabs(I._l), std::fabs(I._u));
}

inline Interval
inv(Interval const& I)
{
  if (I._l <= 0. && I._u >= 0.)
    throw Interval::Exceptions(Interval::Exceptions::INV);
  return Interval(1. / I._u, 1. / I._l);
}

inline Interval
sqr(Interval const& I)
{
  int imid = -1;
  return Interval(mc::sqr(mid(I._l, I._u, 0., imid)),
                  std::max(mc::sqr(I._l), mc::sqr(I._u)));
}

inline Interval
exp(Interval const& I)
{
#ifdef MC__INTERVAL_TRACE
  std::cout << "In exp(Interval const&)\n";
#endif
  return Interval(std::exp(I._l), std::exp(I._u));
}

inline Interval&&
exp(Interval&& I)
{
#ifdef MC__INTERVAL_TRACE
  std::cout << "In exp(Interval &&)\n";
#endif
  I._l = std::exp(I._l);
  I._u = std::exp(I._u);
  return std::move(I);
}

inline Interval
arrh(Interval const& I, double const& a)
{
  return exp(-a / I);
}

inline Interval
log(Interval const& I)
{
  if (I._l <= 0.) throw Interval::Exceptions(Interval::Exceptions::LOG);
  return Interval(std::log(I._l), std::log(I._u));
}

inline Interval
xlog(Interval const& I)
{
  if (I._l < 0.) throw Interval::Exceptions(Interval::Exceptions::LOG);
  int imid = -1;
  return Interval(xlog(mid(I._l, I._u, std::exp(-1.), imid)),
                  std::max(xlog(I._l), xlog(I._u)));
}

// added AVT.SVT 06.06.2017
inline Interval
lmtd(Interval const& I1, Interval const& I2)
{
  if (I1._l <= 0. || I2._l <= 0.)
    throw Interval::Exceptions(Interval::Exceptions::LOG);

  return Interval(lmtd(I1._l, I2._l), lmtd(I1._u, I2._u));
}

// added AVT.SVT 06.06.2017
inline Interval
rlmtd(Interval const& I1, Interval const& I2)
{
  if (I1._l <= 0. || I2._l <= 0.)
    throw Interval::Exceptions(Interval::Exceptions::LOG);

  return Interval(rlmtd(I1._l, I2._l), rlmtd(I1._u, I2._u));
}

inline Interval
erf(Interval const& I)
{
  return Interval(::erf(I._l), ::erf(I._u));
}

inline Interval
erfc(Interval const& I)
{
  return Interval(::erfc(I._u), ::erfc(I._l));
}

inline Interval
sqrt(Interval const& I)
{
  if (I._l < 0.) throw Interval::Exceptions(Interval::Exceptions::SQRT);
  return Interval(std::sqrt(I._l), std::sqrt(I._u));
}

inline Interval
fabs(Interval const& I)
{
  int imid = -1;
  return Interval(std::fabs(mid(I._l, I._u, 0., imid)),
                  std::max(std::fabs(I._l), std::fabs(I._u)));
}

inline Interval
relu(Interval const& I)
{
  return Interval(std::max(I._l, 0.), std::max(I._u, 0.));
}

inline Interval
pow(Interval const& I, const int n)
{
  if (n == 0)
  {
    return 1.;
  }
  if (n == 1)
  {
    return I;
  }
  if (n >= 2 && n % 2 == 0)
  {
    int imid = -1;
    return Interval(std::pow(mid(I._l, I._u, 0., imid), n),
                    std::max(std::pow(I._l, n), std::pow(I._u, n)));
  }
  if (n >= 3)
  {
    return Interval(std::pow(I._l, n), std::pow(I._u, n));
  }
  return inv(pow(I, -n));
}

inline Interval
prod(const unsigned int n, const Interval* I)
{
  switch (n)
  {
    case 0:
      return 1.;
    case 1:
      return I[0];
    default:
      return I[0] * prod(n - 1, I + 1);
  }
}

inline Interval
monom(const unsigned int n, const Interval* I, const unsigned* k)
{
  switch (n)
  {
    case 0:
      return 1.;
    case 1:
      return pow(I[0], (int)k[0]);
    default:
      return pow(I[0], (int)k[0]) * monom(n - 1, I + 1, k + 1);
  }
}

inline Interval
cheb(Interval const& I0, const unsigned n)
{
  Interval I(-1., 1.);
  if (!inter(I, I0, I))
  {
    // if ( I._l < -1.-1e1*machprec() || I._u > 1.+1e1*machprec() ){
    throw typename Interval::Exceptions(Interval::Exceptions::CHEB);
  }
  switch (n)
  {
    case 0:
      return 1.;
    case 1:
      return I;
    case 2:
      return 2. * sqr(I) - 1.;
    default:
    {
      int kL = n - std::ceil(n * std::acos(I._l) / mc::PI);
      if (kL <= 0) kL = 0;
      int kU = n - std::floor(n * std::acos(I._u) / mc::PI);
      if (kU >= (int)n) kU = n;
#ifdef MC__INTERVAL_CHEB_DEBUG
      std::cout << "  kL: " << kL << "  kU: " << kU;
#endif
      if (kU - kL <= 1)
      {  // monotonic part
        double TL = std::cos(n * std::acos(I._l));
        double TU = std::cos(n * std::acos(I._u));
        return (TL <= TU ? Interval(TL, TU) : Interval(TU, TL));
      }
      else if (kU - kL == 2)
      {  // single extremum in range
        double TL = std::cos(n * std::acos(I._l));
        double TU = std::cos(n * std::acos(I._u));
        if ((n - kL) % 2)
          return (TL <= TU ? Interval(TL, 1.) : Interval(TU, 1.));  // minimum
        else
          return (TL <= TU ? Interval(-1., TU) : Interval(-1., TL));  // maximum
      }
      break;
    }
  }
  // Interval Icheb = 2.*I*cheb(I,n-1)-cheb(I,n-2);
  // return( inter( Icheb, Icheb, Interval(-1.,1.) )? Icheb: Interval(-1.,1.) );
  return Interval(-1., 1.);
}

inline Interval
pow(Interval const& I, double const& a)
{
  return exp(a * log(I));
}

inline Interval
pow(Interval const& I1, Interval const& I2)
{
  return exp(I2 * log(I1));
}

inline Interval
hull(Interval const& I1, Interval const& I2)
{
  return Interval(std::min(I1._l, I2._l), std::max(I1._u, I2._u));
}

inline Interval
min(Interval const& I1, Interval const& I2)
{
  return Interval(std::min(I1._l, I2._l), std::min(I1._u, I2._u));
}

inline Interval
max(Interval const& I1, Interval const& I2)
{
  return Interval(std::max(I1._l, I2._l), std::max(I1._u, I2._u));
}

inline Interval
min(const unsigned int n, const Interval* I)
{
  Interval I2(n == 0 || !I ? 0. : I[0]);
  for (unsigned int i = 1; i < n; i++) I2 = min(I2, I[i]);
  return I2;
}

inline Interval
max(const unsigned int n, const Interval* I)
{
  Interval I2(n == 0 || !I ? 0. : I[0]);
  for (unsigned int i = 1; i < n; i++) I2 = max(I2, I[i]);
  return I2;
}

inline Interval
cos(Interval const& I)
{
  const int k    = std::ceil(-(1. + I._l / PI) / 2.);  // -pi <= xL+2*k*pi < pi
  const double l = I._l + 2. * PI * k, u = I._u + 2. * PI * k;
  if (l <= 0)
  {
    if (u <= 0) return Interval(std::cos(l), std::cos(u));
    if (u >= PI) return Interval(-1., 1.);
    return Interval(std::min(std::cos(l), std::cos(u)), 1.);
  }
  if (u <= PI) return Interval(std::cos(u), std::cos(l));
  if (u >= 2. * PI) return Interval(-1., 1.);
  return Interval(-1., std::max(std::cos(l), std::cos(u)));
}

inline Interval
sin(Interval const& I)
{
  return cos(I - PI / 2.);
}

inline Interval
tan(Interval const& I)
{
  const int k    = std::ceil(-0.5 - I._l / PI);  // -pi/2 <= xL+k*pi < pi/2
  const double l = I._l + PI * k, u = I._u + PI * k;
  if (u >= 0.5 * PI) throw Interval::Exceptions(Interval::Exceptions::TAN);
  return Interval(std::tan(l), std::tan(u));
}

inline Interval
acos(Interval const& I)
{
  if (I._l < -1. || I._u > 1.)
    throw Interval::Exceptions(Interval::Exceptions::ACOS);
  return Interval(std::acos(I._u), std::acos(I._l));
}

inline Interval
asin(Interval const& I)
{
  if (I._l < -1. || I._u > 1.)
    throw Interval::Exceptions(Interval::Exceptions::ASIN);
  return Interval(std::asin(I._l), std::asin(I._u));
}

inline Interval
atan(Interval const& I)
{
  return Interval(std::atan(I._l), std::atan(I._u));
}

inline Interval
cosh(Interval const& I)
{
  int imid = -1;
  return Interval(std::cosh(mid(I._l, I._u, 0., imid)),
                  std::max(std::cosh(I._l), std::cosh(I._u)));
}

inline Interval
sinh(Interval const& I)
{
  return Interval(std::sinh(I._l), std::sinh(I._u));
}

inline Interval
tanh(Interval const& I)
{
  return Interval(std::tanh(I._l), std::tanh(I._u));
}

inline Interval
fstep(Interval const& I)
{
  if (I._l >= 0)
    return Interval(1.);
  else if (I._u < 0)
    return Interval(0.);
  return Interval(0., 1.);
}

inline Interval
bstep(Interval const& I)
{
  return fstep(-I);
}

inline std::ostream&
operator<<(std::ostream& out, Interval const& I)
{
  out << std::right << std::scientific
      << std::setprecision(Interval::options.DISPLAY_DIGITS);
  out << "[ " << std::setw(Interval::options.DISPLAY_DIGITS + 7) << I.l()
      << " : " << std::setw(Interval::options.DISPLAY_DIGITS + 7) << I.u()
      << " ]";
  return out;
}

inline bool
inter(Interval& XIY, Interval const& X, Interval const& Y)
{
  if (X._l > Y._u || Y._l > X._u) return false;
  XIY._l = std::max(X._l, Y._l);
  XIY._u = std::min(X._u, Y._u);
  return true;
}

inline bool
operator==(Interval const& I1, Interval const& I2)
{
  return (I1._l == I2._l && I1._u == I2._u);
}

inline bool
operator!=(Interval const& I1, Interval const& I2)
{
  return (I1._l != I2._l || I1._u != I2._u);
}

inline bool
operator<=(Interval const& I1, Interval const& I2)
{
  return (I1._l >= I2._l && I1._u <= I2._u);
}

inline bool
operator>=(Interval const& I1, Interval const& I2)
{
  return (I1._l <= I2._l && I1._u >= I2._u);
}

inline bool
operator<(Interval const& I1, Interval const& I2)
{
  return (I1._l > I2._l && I1._u < I2._u);
}

inline bool
operator>(Interval const& I1, Interval const& I2)
{
  return (I1._l < I2._l && I1._u > I2._u);
}

}  // namespace mc


#ifdef MC__USE_FADBAD

#include "mcfadbad.hpp"
// #include "fadbad.h"

namespace fadbad
{

//! @brief Specialization of the structure fadbad::Op for use of the type
//! mc::Interval of MC++ as a template parameter of the classes fadbad::F,
//! fadbad::B and fadbad::T of FADBAD++
template <>
struct Op<mc::Interval>
{
  typedef double Base;
  typedef mc::Interval T;
  static Base
  myInteger(const int i)
  {
    return Base(i);
  }
  static Base
  myZero()
  {
    return myInteger(0);
  }
  static Base
  myOne()
  {
    return myInteger(1);
  }
  static Base
  myTwo()
  {
    return myInteger(2);
  }
  static double
  myPI()
  {
    return mc::PI;
  }
  static T
  myPos(const T& x)
  {
    return x;
  }
  static T
  myNeg(const T& x)
  {
    return -x;
  }
  template <typename U>
  static T&
  myCadd(T& x, const U& y)
  {
    return x += y;
  }
  template <typename U>
  static T&
  myCsub(T& x, const U& y)
  {
    return x -= y;
  }
  template <typename U>
  static T&
  myCmul(T& x, const U& y)
  {
    return x *= y;
  }
  template <typename U>
  static T&
  myCdiv(T& x, const U& y)
  {
    return x /= y;
  }
  static T
  myInv(const T& x)
  {
    return mc::inv(x);
  }
  static T
  mySqr(const T& x)
  {
    return mc::pow(x, 2);
  }
  template <typename X, typename Y>
  static T
  myPow(const X& x, const Y& y)
  {
    return mc::pow(x, y);
  }
  // static T myCheb( const T& x, const unsigned n ) { return mc::cheb( x, n );
  // }
  static T
  mySqrt(const T& x)
  {
    return mc::sqrt(x);
  }
  static T
  myLog(const T& x)
  {
    return mc::log(x);
  }
  static T
  myExp(const T& x)
  {
    return mc::exp(x);
  }
  static T
  mySin(const T& x)
  {
    return mc::sin(x);
  }
  static T
  myCos(const T& x)
  {
    return mc::cos(x);
  }
  static T
  myTan(const T& x)
  {
    return mc::tan(x);
  }
  static T
  myAsin(const T& x)
  {
    return mc::asin(x);
  }
  static T
  myAcos(const T& x)
  {
    return mc::acos(x);
  }
  static T
  myAtan(const T& x)
  {
    return mc::atan(x);
  }
  static T
  mySinh(const T& x)
  {
    return mc::sinh(x);
  }
  static T
  myCosh(const T& x)
  {
    return mc::cosh(x);
  }
  static T
  myTanh(const T& x)
  {
    return mc::tanh(x);
  }
  static bool
  myEq(const T& x, const T& y)
  {
    return x == y;
  }
  static bool
  myNe(const T& x, const T& y)
  {
    return x != y;
  }
  static bool
  myLt(const T& x, const T& y)
  {
    return x < y;
  }
  static bool
  myLe(const T& x, const T& y)
  {
    return x <= y;
  }
  static bool
  myGt(const T& x, const T& y)
  {
    return x > y;
  }
  static bool
  myGe(const T& x, const T& y)
  {
    return x >= y;
  }
};

}  // end namespace fadbad

#endif // MC__USE_FADBAD


#include "mcop.hpp"

namespace mc
{

//! @brief Specialization of the structure mc::Op to allow usage of the type
//! mc::Interval for DAG evaluation or as a template parameter in other MC++
//! classes
template <>
struct Op<mc::Interval>
{
  typedef mc::Interval T;
  static T
  point(double const& c)
  {
    return T(c);
  }
  static T
  zeroone()
  {
    return T(0., 1.);
  }
  static void
  I(T& x, const T& y)
  {
    x = y;
  }
  static double
  l(const T& x)
  {
    return x.l();
  }
  static double
  u(const T& x)
  {
    return x.u();
  }
  static double
  abs(const T& x)
  {
    return mc::abs(x);
  }
  static double
  mid(const T& x)
  {
    return mc::mid(x);
  }
  static double
  diam(const T& x)
  {
    return mc::diam(x);
  }
  static T
  inv(const T& x)
  {
    return mc::inv(x);
  }
  static T
  sqr(const T& x)
  {
    return mc::sqr(x);
  }
  static T
  sqrt(const T& x)
  {
    return mc::sqrt(x);
  }
  static T
  exp(const T& x)
  {
    return mc::exp(x);
  }
  static T
  log(const T& x)
  {
    return mc::log(x);
  }
  static T
  xlog(const T& x)
  {
    return mc::xlog(x);
  }
  static T
  fabs(const T& x)
  {
    return mc::fabs(x);
  }
  static T
  sin(const T& x)
  {
    return mc::sin(x);
  }
  static T
  cos(const T& x)
  {
    return mc::cos(x);
  }
  static T
  tan(const T& x)
  {
    return mc::tan(x);
  }
  static T
  asin(const T& x)
  {
    return mc::asin(x);
  }
  static T
  acos(const T& x)
  {
    return mc::acos(x);
  }
  static T
  atan(const T& x)
  {
    return mc::atan(x);
  }
  static T
  sinh(const T& x)
  {
    return mc::sinh(x);
  }
  static T
  cosh(const T& x)
  {
    return mc::cosh(x);
  }
  static T
  tanh(const T& x)
  {
    return mc::tanh(x);
  }
  static T
  erf(const T& x)
  {
    return mc::erf(x);
  }
  static T
  erfc(const T& x)
  {
    return mc::erfc(x);
  }
  static T
  fstep(const T& x)
  {
    return mc::fstep(x);
  }
  static T
  bstep(const T& x)
  {
    return mc::bstep(x);
  }
  static T
  lmtd(const T& x, const T& y)
  {
    return mc::lmtd(x, y);
  }
  static T
  rlmtd(const T& x, const T& y)
  {
    return mc::rlmtd(x, y);
  }
  static T
  min(const T& x, const T& y)
  {
    return mc::min(x, y);
  }
  static T
  max(const T& x, const T& y)
  {
    return mc::max(x, y);
  }
  template <typename X, typename Y>
  static T
  pow(const X& x, const Y& y)
  {
    return mc::pow(x, y);
  }
  static T
  cheb(const T& x, const unsigned n)
  {
    return mc::cheb(x, n);
  }
  static T
  prod(const unsigned int n, const T* x)
  {
    return mc::prod(n, x);
  }
  static T
  monom(const unsigned int n, const T* x, const unsigned* k)
  {
    return mc::monom(n, x, k);
  }
  static T
  hull(const T& x, const T& y)
  {
    return mc::hull(x, y);
  }
  static bool
  inter(T& xIy, const T& x, const T& y)
  {
    return mc::inter(xIy, x, y);
  }
  static bool
  eq(const T& x, const T& y)
  {
    return x == y;
  }
  static bool
  ne(const T& x, const T& y)
  {
    return x != y;
  }
  static bool
  lt(const T& x, const T& y)
  {
    return x < y;
  }
  static bool
  le(const T& x, const T& y)
  {
    return x <= y;
  }
  static bool
  gt(const T& x, const T& y)
  {
    return x > y;
  }
  static bool
  ge(const T& x, const T& y)
  {
    return x >= y;
  }
};

}  // namespace mc

#endif
