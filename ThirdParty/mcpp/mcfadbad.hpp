// Copyright (C) 2009-2016 Benoit Chachuat, Imperial College London.
// All Rights Reserved.
// This code is published under the Eclipse Public License.

#ifndef MC__MCFADBAD_HPP
#define MC__MCFADBAD_HPP

#include "interval.hpp"
#include "mccormick.hpp"
#include "tmodel.hpp"
#include "cmodel.hpp"
#include "specbnd.hpp"

#include "fadbad.h"

namespace fadbad
{

//! @brief Specialization of the structure fadbad::Op for use of the type mc::Interval of MC++ as a template parameter of the classes fadbad::F, fadbad::B and fadbad::T of FADBAD++
template <> struct Op<mc::Interval>
{
  typedef double Base;
  typedef mc::Interval I;
  static Base myInteger( const int i ) { return Base(i); }
  static Base myZero() { return myInteger(0); }
  static Base myOne() { return myInteger(1);}
  static Base myTwo() { return myInteger(2); }
  static double myPI() { return mc::PI; }
  static I myPos( const I& x ) { return  x; }
  static I myNeg( const I& x ) { return -x; }
  template <typename U> static I& myCadd( I& x, const U& y ) { return x+=y; }
  template <typename U> static I& myCsub( I& x, const U& y ) { return x-=y; }
  template <typename U> static I& myCmul( I& x, const U& y ) { return x*=y; }
  template <typename U> static I& myCdiv( I& x, const U& y ) { return x/=y; }
  static I myInv( const I& x ) { return mc::inv( x ); }
  static I mySqr( const I& x ) { return mc::pow( x, 2 ); }
  template <typename X, typename Y> static I myPow( const X& x, const Y& y ) { return mc::pow( x, y ); }
  static I myCheb( const I& x, const unsigned n ) { return mc::cheb( x, n ); }
  static I mySqrt( const I& x ) { return mc::sqrt( x ); }
  static I myLog( const I& x ) { return mc::log( x ); }
  static I myExp( const I& x ) { return mc::exp( x ); }
  static I mySin( const I& x ) { return mc::sin( x ); }
  static I myCos( const I& x ) { return mc::cos( x ); }
  static I myTan( const I& x ) { return mc::tan( x ); }
  static I myAsin( const I& x ) { return mc::asin( x ); }
  static I myAcos( const I& x ) { return mc::acos( x ); }
  static I myAtan( const I& x ) { return mc::atan( x ); }
  static bool myEq( const I& x, const I& y ) { return x==y; }
  static bool myNe( const I& x, const I& y ) { return x!=y; }
  static bool myLt( const I& x, const I& y ) { return x<y; }
  static bool myLe( const I& x, const I& y ) { return x<=y; }
  static bool myGt( const I& x, const I& y ) { return x>y; }
  static bool myGe( const I& x, const I& y ) { return x>=y; }
};

//! @brief Specialization of the structure fadbad::Op to allow usage of the type mc::McCormick of MC++ as a template parameter of the classes fadbad::F, fadbad::B and fadbad::T of FADBAD++
template<typename T> struct Op< mc::McCormick<T> >
{
  typedef mc::McCormick<T> MC;
  typedef double Base;
  static Base myInteger( const int i ) { return Base(i); }
  static Base myZero() { return myInteger(0); }
  static Base myOne() { return myInteger(1);}
  static Base myTwo() { return myInteger(2); }
  static double myPI() { return mc::PI; }
  static MC myPos( const MC& x ) { return  x; }
  static MC myNeg( const MC& x ) { return -x; }
  template <typename U> static MC& myCadd( MC& x, const U& y ) { return x+=y; }
  template <typename U> static MC& myCsub( MC& x, const U& y ) { return x-=y; }
  template <typename U> static MC& myCmul( MC& x, const U& y ) { return x*=y; }
  template <typename U> static MC& myCdiv( MC& x, const U& y ) { return x/=y; }
  static MC myInv( const MC& x ) { return mc::inv( x ); }
  static MC mySqr( const MC& x ) { return mc::pow( x, 2 ); }
  template <typename X, typename Y> static MC myPow( const X& x, const Y& y ) { return mc::pow( x, y ); }
  static MC myCheb( const MC& x, const unsigned n ) { return mc::cheb( x, n ); }
  static MC mySqrt( const MC& x ) { return mc::sqrt( x ); }
  static MC myLog( const MC& x ) { return mc::log( x ); }
  static MC myExp( const MC& x ) { return mc::exp( x ); }
  static MC mySin( const MC& x ) { return mc::sin( x ); }
  static MC myCos( const MC& x ) { return mc::cos( x ); }
  static MC myTan( const MC& x ) { return mc::tan( x ); }
  static MC myAsin( const MC& x ) { return mc::asin( x ); }
  static MC myAcos( const MC& x ) { return mc::acos( x ); }
  static MC myAtan( const MC& x ) { return mc::atan( x ); }
  static bool myEq( const MC& x, const MC& y ) { return x==y; }
  static bool myNe( const MC& x, const MC& y ) { return x!=y; }
  static bool myLt( const MC& x, const MC& y ) { return x<y; }
  static bool myLe( const MC& x, const MC& y ) { return x<=y; }
  static bool myGt( const MC& x, const MC& y ) { return x>y; }
  static bool myGe( const MC& x, const MC& y ) { return x>=y; }
};
    
//! @brief Specialization of the structure fadbad::Op for use of the type mc::TVar of MC++ as a template parameter of the classes fadbad::F, fadbad::B and fadbad::T of FADBAD++
template< typename T > struct Op< mc::TVar<T> >
{ 
  typedef mc::TVar<T> TM;
  typedef double Base;
  static Base myInteger( const int i ) { return Base(i); }
  static Base myZero() { return myInteger(0); }
  static Base myOne() { return myInteger(1);}
  static Base myTwo() { return myInteger(2); }
  static double myPI() { return mc::PI; }
  static TM myPos( const TM& x ) { return  x; }
  static TM myNeg( const TM& x ) { return -x; }
  template <typename U> static TM& myCadd( TM& x, const U& y ) { return x+=y; }
  template <typename U> static TM& myCsub( TM& x, const U& y ) { return x-=y; }
  template <typename U> static TM& myCmul( TM& x, const U& y ) { return x*=y; }
  template <typename U> static TM& myCdiv( TM& x, const U& y ) { return x/=y; }
  static TM myInv( const TM& x ) { return mc::inv( x ); }
  static TM mySqr( const TM& x ) { return mc::pow( x, 2 ); }
  template <typename X, typename Y> static TM myPow( const X& x, const Y& y ) { return mc::pow( x, y ); }
  static TM myCheb( const TM& x, const unsigned n ) { return mc::cheb( x, n ); }
  static TM mySqrt( const TM& x ) { return mc::sqrt( x ); }
  static TM myLog( const TM& x ) { return mc::log( x ); }
  static TM myExp( const TM& x ) { return mc::exp( x ); }
  static TM mySin( const TM& x ) { return mc::sin( x ); }
  static TM myCos( const TM& x ) { return mc::cos( x ); }
  static TM myTan( const TM& x ) { return mc::tan( x ); }
  static TM myAsin( const TM& x ) { return mc::asin( x ); }
  static TM myAcos( const TM& x ) { return mc::acos( x ); }
  static TM myAtan( const TM& x ) { return mc::atan( x ); }
  static bool myEq( const TM& x, const TM& y ) { return mc::Op<T>::eq(const_cast<TM*>(&x)->bound(),const_cast<TM*>(&y)->bound()); } 
  static bool myNe( const TM& x, const TM& y ) { return mc::Op<T>::ne(const_cast<TM*>(&x)->bound(),const_cast<TM*>(&y)->bound()); }
  static bool myLt( const TM& x, const TM& y ) { return mc::Op<T>::lt(const_cast<TM*>(&x)->bound(),const_cast<TM*>(&y)->bound()); }
  static bool myLe( const TM& x, const TM& y ) { return mc::Op<T>::le(const_cast<TM*>(&x)->bound(),const_cast<TM*>(&y)->bound()); }
  static bool myGt( const TM& x, const TM& y ) { return mc::Op<T>::gt(const_cast<TM*>(&x)->bound(),const_cast<TM*>(&y)->bound()); }
  static bool myGe( const TM& x, const TM& y ) { return mc::Op<T>::ge(const_cast<TM*>(&x)->bound(),const_cast<TM*>(&y)->bound()); }
};
     
//! @brief Specialization of the structure fadbad::Op for use of the type mc::CVar of MC++ as a template parameter of the classes fadbad::F, fadbad::B and fadbad::T of FADBAD++
template< typename T > struct Op< mc::CVar<T> >
{ 
  typedef mc::CVar<T> CM;
  typedef double Base;
  static Base myInteger( const int i ) { return Base(i); }
  static Base myZero() { return myInteger(0); }
  static Base myOne() { return myInteger(1);}
  static Base myTwo() { return myInteger(2); }
  static double myPI() { return mc::PI; }
  static CM myPos( const CM& x ) { return  x; }
  static CM myNeg( const CM& x ) { return -x; }
  template <typename U> static CM& myCadd( CM& x, const U& y ) { return x+=y; }
  template <typename U> static CM& myCsub( CM& x, const U& y ) { return x-=y; }
  template <typename U> static CM& myCmul( CM& x, const U& y ) { return x*=y; }
  template <typename U> static CM& myCdiv( CM& x, const U& y ) { return x/=y; }
  static CM myInv( const CM& x ) { return mc::inv( x ); }
  static CM mySqr( const CM& x ) { return mc::pow( x, 2 ); }
  template <typename X, typename Y> static CM myPow( const X& x, const Y& y ) { return mc::pow( x, y ); }
  static CM myCheb( const CM& x, const unsigned n ) { return mc::cheb( x, n ); }
  static CM mySqrt( const CM& x ) { return mc::sqrt( x ); }
  static CM myLog( const CM& x ) { return mc::log( x ); }
  static CM myExp( const CM& x ) { return mc::exp( x ); }
  static CM mySin( const CM& x ) { return mc::sin( x ); }
  static CM myCos( const CM& x ) { return mc::cos( x ); }
  static CM myTan( const CM& x ) { return mc::tan( x ); }
  static CM myAsin( const CM& x ) { return mc::asin( x ); }
  static CM myAcos( const CM& x ) { return mc::acos( x ); }
  static CM myAtan( const CM& x ) { return mc::atan( x ); }
  static bool myEq( const CM& x, const CM& y ) { return mc::Op<T>::eq(const_cast<CM*>(&x)->bound(),const_cast<CM*>(&y)->bound()); } 
  static bool myNe( const CM& x, const CM& y ) { return mc::Op<T>::ne(const_cast<CM*>(&x)->bound(),const_cast<CM*>(&y)->bound()); }
  static bool myLt( const CM& x, const CM& y ) { return mc::Op<T>::lt(const_cast<CM*>(&x)->bound(),const_cast<CM*>(&y)->bound()); }
  static bool myLe( const CM& x, const CM& y ) { return mc::Op<T>::le(const_cast<CM*>(&x)->bound(),const_cast<CM*>(&y)->bound()); }
  static bool myGt( const CM& x, const CM& y ) { return mc::Op<T>::gt(const_cast<CM*>(&x)->bound(),const_cast<CM*>(&y)->bound()); }
  static bool myGe( const CM& x, const CM& y ) { return mc::Op<T>::ge(const_cast<CM*>(&x)->bound(),const_cast<CM*>(&y)->bound()); }
};
 
//! @brief Specialization of the structure fadbad::Op for use of the type mc::Specbnd of MC++ as a template parameter of the classes fadbad::F, fadbad::B and fadbad::T of FADBAD++
template<typename T> struct Op< mc::Specbnd<T> >{ 
  typedef mc::Specbnd<T> SB;
  typedef double Base;
  static Base myInteger( const int i ) { return Base(i); }
  static Base myZero() { return myInteger(0); }
  static Base myOne() { return myInteger(1);}
  static Base myTwo() { return myInteger(2); }
  static double myPI() { return mc::PI; }
  static SB myPos( const SB& x ) { return  x; }
  static SB myNeg( const SB& x ) { return -x; }
  template <typename U> static SB& myCadd( SB& x, const U& y ) { return x+=y; }
  template <typename U> static SB& myCsub( SB& x, const U& y ) { return x-=y; }
  template <typename U> static SB& myCmul( SB& x, const U& y ) { return x*=y; }
  template <typename U> static SB& myCdiv( SB& x, const U& y ) { return x/=y; }
  static SB myInv( const SB& x ) { return mc::inv( x ); }
  static SB mySqr( const SB& x ) { return mc::sqr( x ); }
  template <typename X, typename Y> static SB myPow( const X& x, const Y& y ) { return mc::pow( x, y ); }
  static SB myCheb( const SB& x, const unsigned n ) { return mc::cheb( x, n ); }
  static SB mySqrt( const SB& x ) { return mc::sqrt( x ); }
  static SB myLog( const SB& x ) { return mc::log( x ); }
  static SB myExp( const SB& x ) { return mc::exp( x ); }
  static SB mySin( const SB& x ) { return mc::sin( x ); }
  static SB myCos( const SB& x ) { return mc::cos( x ); }
  static SB myTan( const SB& x ) { return mc::tan( x ); }
  static SB myAsin( const SB& x ) { return mc::asin( x ); }
  static SB myAcos( const SB& x ) { return mc::acos( x ); }
  static SB myAtan( const SB& x ) { return mc::atan( x ); }
  static bool myEq( const SB& x, const SB& y ) { return mc::Op<T>::eq(x.SI(),y.SI()); } 
  static bool myNe( const SB& x, const SB& y ) { return mc::Op<T>::ne(x.SI(),y.SI()); }
  static bool myLt( const SB& x, const SB& y ) { return mc::Op<T>::lt(x.SI(),y.SI()); }
  static bool myLe( const SB& x, const SB& y ) { return mc::Op<T>::le(x.SI(),y.SI()); }
  static bool myGt( const SB& x, const SB& y ) { return mc::Op<T>::gt(x.SI(),y.SI()); }
  static bool myGe( const SB& x, const SB& y ) { return mc::Op<T>::ge(x.SI(),y.SI()); }
};

} // end namespace fadbad

#include "fadiff.h"

namespace fadbad
{

template <typename T, unsigned int N>
INLINE2 FTypeName<T,N> pow2(const FTypeName<T,N>& a, const int b)
{
	FTypeName<T,N> c(Op<T>::myPow(a.val(),b));
	if (!a.depend()) return c;
	T tmp(b*Op<T>::myPow(a.val(),b-1));
	c.setDepend(a);
	for(unsigned int i=0;i<N;++i) c[i]=tmp*a[i];
	return c;
}

template <typename T >
INLINE2 FTypeName<T,0> pow2(const FTypeName<T,0>& a, const int b)
{
	FTypeName<T,0> c(Op<T>::myPow(a.val(),b));
	if (!a.depend()) return c;
	T tmp(b*Op<T>::myPow(a.val(),b-1));
	c.setDepend(a);
	for(unsigned int i=0;i<c.size();++i) c[i]=tmp*a[i];
	return c;
}

template <typename T, unsigned int N>
INLINE2 FTypeName<T,N> cheb(const FTypeName<T,N>& a, const unsigned b)
{
	FTypeName<T,N> c(Op<T>::myCheb(a.val(),b));
	if (!a.depend()) return c;
        // dTn/dx = n*Un-1(x)
        // Un-1(x) = 2*(T1(x)+T3(x)+...+Tn-1(x)) if n even
        //           2*(T0(x)+T2(x)+...+Tn-1(x))-1 if n odd
	T tmp(0.);
        if( b%2 ){ // odd case
          for( unsigned k=0; k<b; k+=2 ) tmp += Op<T>::myCheb(a.val(),k);
          tmp *= 2.; tmp -= 1.;
        }
        else{ // even case
          for( unsigned k=1; k<b; k+=2 ) tmp += Op<T>::myCheb(a.val(),k);
          tmp *= 2.;
        }
	c.setDepend(a);
	for(unsigned int i=0;i<N;++i) c[i]=tmp*a[i];
	return c;
}

template <typename T>
INLINE2 FTypeName<T,0> cheb(const FTypeName<T,0>& a, const unsigned b)
{
	FTypeName<T,0> c(Op<T>::myCheb(a.val(),b));
	if (!a.depend()) return c;
        // dTn/dx = n*Un-1(x)
        // Un-1(x) = 2*(T1(x)+T3(x)+...+Tn-1(x)) if n even
        //           2*(T0(x)+T2(x)+...+Tn-1(x))-1 if n odd
	T tmp(0.);
        if( b%2 ){ // odd case
          for( unsigned k=0; k<b; k+=2 ) tmp += Op<T>::myCheb(a.val(),k);
          tmp *= 2.; tmp -= 1.;
        }
        else{ // even case
          for( unsigned k=1; k<b; k+=2 ) tmp += Op<T>::myCheb(a.val(),k);
          tmp *= 2.;
        }
	c.setDepend(a);
	for(unsigned int i=0;i<c.size();++i) c[i]=tmp*a[i];
	return c;
}

} // end namespace fadbad

#include "tadiff.h"

namespace fadbad
{

template <typename U, int N>
struct TTypeNamePOW3 : public UnTTypeNameHV<U,N>
{
	int m_b;
	TTypeNamePOW3(const U& val, TTypeNameHV<U,N>* pOp, const int b):UnTTypeNameHV<U,N>(val,pOp),m_b(b){}
	TTypeNamePOW3(TTypeNameHV<U,N>* pOp, const int b):UnTTypeNameHV<U,N>(pOp),m_b(b){}
	unsigned int eval(const unsigned int k)
	{
		unsigned int l=this->opEval(k);
                if( m_b==0 ){
 			if (0==this->length()) { this->val(0)=Op<U>::myOne(); this->length()=1; }
			for(unsigned int i=this->length();i<l;++i) { this->val(i)=Op<U>::myZero(); }
                }
                else if( m_b==1 ){
			for(unsigned int i=this->length();i<l;++i) { this->val(i)=this->opVal(i); }
		}
                else if( m_b==2 ){
			if (0==this->length()) { this->val(0)=Op<U>::mySqr(this->opVal(0)); this->length()=1; }
			for(unsigned int i=this->length();i<l;++i)
			{
				this->val(i)=Op<U>::myZero();
				unsigned int m=(i+1)/2;
				for(unsigned int j=0;j<m;++j) Op<U>::myCadd(this->val(i), this->opVal(i-j)*this->opVal(j));
				Op<U>::myCmul(this->val(i), Op<U>::myTwo());
				if (0==i%2) Op<U>::myCadd(this->val(i), Op<U>::mySqr(this->opVal(m)));
			}
		}
		else if( m_b==3 ){
			if (0==this->length()) { this->val(0)=Op<U>::myPow(this->opVal(0),m_b); this->length()=1; }
			if (1<l && 1==this->length() ) { this->val(1)=Op<U>::myPow(this->opVal(0),m_b-1)
				*this->opVal(1)*Op<U>::myInteger(m_b); this->length()=2; }
			if (2<l && 2==this->length() ) { this->val(2)=Op<U>::myPow(this->opVal(0),m_b-2)
				*( this->opVal(0)*this->opVal(2) + Op<U>::myInteger(m_b-1)*Op<U>::mySqr(this->opVal(1)) )
				*Op<U>::myInteger(m_b); this->length()=3; }
			for(unsigned int i=this->length();i<l;++i)
                        {
                                this->val(i)=Op<U>::myZero();
				unsigned int m=(i+1)/2;
				for(unsigned int j=0;j<m;++j) Op<U>::myCadd(this->val(i), this->opVal(i-j)*this->opVal(j));
				Op<U>::myCmul(this->val(i), Op<U>::myTwo());
				if (0==i%2) Op<U>::myCadd(this->val(i), Op<U>::mySqr(this->opVal(m)));                        
                        }
			for(unsigned int i=l-1; i>=this->length();--i)
			{
				Op<U>::myCmul(this->val(i), this->opVal(0));
				for(unsigned int j=1;j<=i;++j) Op<U>::myCadd(this->val(i), this->val(i-j)*this->opVal(j));
			}
		}
                else{                       
			if (0==this->length()) { this->val(0)=Op<U>::myPow(this->opVal(0),m_b); this->length()=1; }
			for(unsigned int i=this->length();i<l;++i)
			{
				this->val(i)=Op<U>::myZero();
				for(unsigned int j=0;j<i;++j)
					Op<U>::myCadd(this->val(i), ( m_b - (m_b+Op<U>::myOne()) * Op<U>::myInteger(j) /
						Op<U>::myInteger(i) )*this->opVal(i-j)*this->val(j));
			}
                }
		return this->length()=l;
	}
private:
	void operator=(const TTypeNamePOW3<U,N>&){} // not allowed
};

template <typename U, int N>
TTypeName<U,N> pow(const TTypeName<U,N>& val, const int b)
{
	TTypeNameHV<U,N>* pHV=val.length()>0 ?
		new TTypeNamePOW3<U,N>(Op<U>::myPow(val.val(),b), val.getTTypeNameHV(), b):
		new TTypeNamePOW3<U,N>(val.getTTypeNameHV(), b);
	return TTypeName<U,N>(pHV);
}

} // end namespace fadbad

#include "badiff.h"

#include "mcop.hpp"

namespace mc
{

//! @brief C++ structure for specialization of the mc::Op templated structure for use of the FADBAD type fadbad::F inside other MC++ types
template< typename U > struct Op< fadbad::F<U> >
{
  typedef fadbad::F<U> TU;
  static TU point( const double c ) { throw std::runtime_error("mc::Op<fadbad::F<U>>::point -- operation not permitted"); }
  static TU zeroone() { throw std::runtime_error("mc::Op<fadbad::F<U>>::zeroone -- operation not permitted"); }
  static void I(TU& x, const TU&y) { x = y; }
  static double l(const TU& x) { throw std::runtime_error("mc::Op<fadbad::F<U>>::l -- operation not permitted"); }
  static double u(const TU& x) { throw std::runtime_error("mc::Op<fadbad::F<U>>::u -- operation not permitted"); }
  static double abs (const TU& x) { throw std::runtime_error("mc::Op<fadbad::F<U>>::abs -- operation not permitted"); }
  static double mid (const TU& x) { throw std::runtime_error("mc::Op<fadbad::F<U>>::mid -- operation not permitted"); }
  static double diam(const TU& x) { throw std::runtime_error("mc::Op<fadbad::F<U>>::diam -- operation not permitted"); }
  static TU inv (const TU& x) { return 1./x;  }
  static TU sqr (const TU& x) { return fadbad::sqr(x);  }
  static TU sqrt(const TU& x) { return fadbad::sqrt(x); }
  static TU log (const TU& x) { return fadbad::log(x);  }
  static TU xlog(const TU& x) { return x*fadbad::log(x); }
  static TU fabs(const TU& x) { throw std::runtime_error("mc::Op<fadbad::F<U>>::fabs -- operation not permitted"); }
  static TU exp (const TU& x) { return fadbad::exp(x);  }
  static TU sin (const TU& x) { return fadbad::sin(x);  }
  static TU cos (const TU& x) { return fadbad::cos(x);  }
  static TU tan (const TU& x) { return fadbad::tan(x);  }
  static TU asin(const TU& x) { return fadbad::asin(x); }
  static TU acos(const TU& x) { return fadbad::acos(x); }
  static TU atan(const TU& x) { return fadbad::atan(x); }
  static TU erf (const TU& x) { throw std::runtime_error("mc::Op<fadbad::F<U>>::erf -- operation not permitted"); }
  static TU erfc(const TU& x) { throw std::runtime_error("mc::Op<fadbad::F<U>>::erfc -- operation not permitted"); }
  static TU fstep(const TU& x) { throw std::runtime_error("mc::Op<fadbad::F<U>>::fstep -- operation not permitted"); }
  static TU bstep(const TU& x) { throw std::runtime_error("mc::Op<fadbad::F<U>>::bstep -- operation not permitted"); }
  static TU hull(const TU& x, const TU& y) { throw std::runtime_error("mc::Op<fadbad::F<U>>::hull -- operation not permitted"); }
  static TU min (const TU& x, const TU& y) { throw std::runtime_error("mc::Op<fadbad::F<U>>::min -- operation not permitted"); }
  static TU max (const TU& x, const TU& y) { throw std::runtime_error("mc::Op<fadbad::F<U>>::max -- operation not permitted"); }
  static TU arh (const TU& x, const double k) { return fadbad::exp(-k/x); }
  static TU cheb(const TU& x, const unsigned n) { return fadbad::cheb(x,n); }
  template <typename X, typename Y> static TU pow(const X& x, const Y& y) { return fadbad::pow(x,y); }
  static TU monomial (const unsigned int n, const U* x, const int* k) { throw std::runtime_error("mc::Op<T<U>>::monomial -- operation not permitted"); }
  static bool inter(TU& xIy, const TU& x, const TU& y) { xIy = x; return true; }
  static bool eq(const TU& x, const TU& y) { throw std::runtime_error("mc::Op<fadbad::F<U>>::eq -- operation not permitted"); }
  static bool ne(const TU& x, const TU& y) { throw std::runtime_error("mc::Op<fadbad::F<U>>::ne -- operation not permitted"); }
  static bool lt(const TU& x, const TU& y) { throw std::runtime_error("mc::Op<fadbad::F<U>>::lt -- operation not permitted"); }
  static bool le(const TU& x, const TU& y) { throw std::runtime_error("mc::Op<fadbad::F<U>>::le -- operation not permitted"); }
  static bool gt(const TU& x, const TU& y) { throw std::runtime_error("mc::Op<fadbad::F<U>>::gt -- operation not permitted"); }
  static bool ge(const TU& x, const TU& y) { throw std::runtime_error("mc::Op<fadbad::F<U>>::ge -- operation not permitted"); }
};

//! @brief C++ structure for specialization of the mc::Op templated structure for use of the FADBAD type fadbad::B inside other MC++ types
template< typename U > struct Op< fadbad::B<U> >
{
  typedef fadbad::B<U> TU;
  static TU point( const double c ) { throw std::runtime_error("mc::Op<fadbad::B<U>>::point -- operation not permitted"); }
  static TU zeroone() { throw std::runtime_error("mc::Op<fadbad::B<U>>::zeroone -- operation not permitted"); }
  static void I(TU& x, const TU&y) { x = y; }
  static double l(const TU& x) { throw std::runtime_error("mc::Op<fadbad::B<U>>::l -- operation not permitted"); }
  static double u(const TU& x) { throw std::runtime_error("mc::Op<fadbad::B<U>>::u -- operation not permitted"); }
  static double abs (const TU& x) { throw std::runtime_error("mc::Op<fadbad::B<U>>::abs -- operation not permitted"); }
  static double mid (const TU& x) { throw std::runtime_error("mc::Op<fadbad::B<U>>::mid -- operation not permitted"); }
  static double diam(const TU& x) { throw std::runtime_error("mc::Op<fadbad::B<U>>::diam -- operation not permitted"); }
  static TU inv (const TU& x) { return 1./x;  }
  static TU sqr (const TU& x) { return fadbad::sqr(x);  }
  static TU sqrt(const TU& x) { return fadbad::sqrt(x); }
  static TU log (const TU& x) { return fadbad::log(x);  }
  static TU xlog(const TU& x) { return x*fadbad::log(x); }
  static TU fabs(const TU& x) { throw std::runtime_error("mc::Op<fadbad::B<U>>::fabs -- operation not permitted"); }
  static TU exp (const TU& x) { return fadbad::exp(x);  }
  static TU sin (const TU& x) { return fadbad::sin(x);  }
  static TU cos (const TU& x) { return fadbad::cos(x);  }
  static TU tan (const TU& x) { return fadbad::tan(x);  }
  static TU asin(const TU& x) { return fadbad::asin(x); }
  static TU acos(const TU& x) { return fadbad::acos(x); }
  static TU atan(const TU& x) { return fadbad::atan(x); }
  static TU erf (const TU& x) { throw std::runtime_error("mc::Op<fadbad::B<U>>::erf -- operation not permitted"); }
  static TU erfc(const TU& x) { throw std::runtime_error("mc::Op<fadbad::B<U>>::erfc -- operation not permitted"); }
  static TU fstep(const TU& x) { throw std::runtime_error("mc::Op<fadbad::B<U>>::fstep -- operation not permitted"); }
  static TU bstep(const TU& x) { throw std::runtime_error("mc::Op<fadbad::B<U>>::bstep -- operation not permitted"); }
  static TU hull(const TU& x, const TU& y) { throw std::runtime_error("mc::Op<fadbad::B<U>>::hull -- operation not permitted"); }
  static TU min (const TU& x, const TU& y) { throw std::runtime_error("mc::Op<fadbad::B<U>>::min -- operation not permitted"); }
  static TU max (const TU& x, const TU& y) { throw std::runtime_error("mc::Op<fadbad::B<U>>::max -- operation not permitted"); }
  static TU arh (const TU& x, const double k) { return fadbad::exp(-k/x); }
  static TU cheb(const TU& x, const unsigned n) { throw std::runtime_error("mc::Op<fadbad::B<U>>::max -- operation not permitted"); }
  template <typename X, typename Y> static TU pow(const X& x, const Y& y) { return fadbad::pow(x,y); }
  static TU monomial (const unsigned int n, const U* x, const int* k) { throw std::runtime_error("mc::Op<fadbad::B<U>>::monomial -- operation not permitted"); }
  static bool inter(TU& xIy, const TU& x, const TU& y) { xIy = x; return true; }
  static bool eq(const TU& x, const TU& y) { throw std::runtime_error("mc::Op<fadbad::B<U>>::eq -- operation not permitted"); }
  static bool ne(const TU& x, const TU& y) { throw std::runtime_error("mc::Op<fadbad::B<U>>::ne -- operation not permitted"); }
  static bool lt(const TU& x, const TU& y) { throw std::runtime_error("mc::Op<fadbad::B<U>>::lt -- operation not permitted"); }
  static bool le(const TU& x, const TU& y) { throw std::runtime_error("mc::Op<fadbad::B<U>>::le -- operation not permitted"); }
  static bool gt(const TU& x, const TU& y) { throw std::runtime_error("mc::Op<fadbad::B<U>>::gt -- operation not permitted"); }
  static bool ge(const TU& x, const TU& y) { throw std::runtime_error("mc::Op<fadbad::B<U>>::ge -- operation not permitted"); }
};

//! @brief C++ structure for specialization of the mc::Op templated structure for use of the FADBAD type fadbad::T inside other MC++ types
template< typename U > struct Op< fadbad::T<U> >
{
  typedef fadbad::T<U> TU;
  static TU point( const double c ) { throw std::runtime_error("mc::Op<fadbad::T<U>>::point -- operation not permitted"); }
  static TU zeroone() { throw std::runtime_error("mc::Op<fadbad::T<U>>::zeroone -- operation not permitted"); }
  static void I(TU& x, const TU&y) { x = y; }
  static double l(const TU& x) { throw std::runtime_error("mc::Op<fadbad::T<U>>::l -- operation not permitted"); }
  static double u(const TU& x) { throw std::runtime_error("mc::Op<fadbad::T<U>>::u -- operation not permitted"); }
  static double abs (const TU& x) { throw std::runtime_error("mc::Op<fadbad::T<U>>::abs -- operation not permitted"); }
  static double mid (const TU& x) { throw std::runtime_error("mc::Op<fadbad::T<U>>::mid -- operation not permitted"); }
  static double diam(const TU& x) { throw std::runtime_error("mc::Op<fadbad::T<U>>::diam -- operation not permitted"); }
  static TU inv (const TU& x) { return 1./x;  }
  static TU sqr (const TU& x) { return fadbad::sqr(x);  }
  static TU sqrt(const TU& x) { return fadbad::sqrt(x); }
  static TU log (const TU& x) { return fadbad::log(x);  }
  static TU xlog(const TU& x) { return x*fadbad::log(x); }
  static TU fabs(const TU& x) { throw std::runtime_error("mc::Op<fadbad::T<U>>::fabs -- operation not permitted"); }
  static TU exp (const TU& x) { return fadbad::exp(x);  }
  static TU sin (const TU& x) { return fadbad::sin(x);  }
  static TU cos (const TU& x) { return fadbad::cos(x);  }
  static TU tan (const TU& x) { return fadbad::tan(x);  }
  static TU asin(const TU& x) { return fadbad::asin(x); }
  static TU acos(const TU& x) { return fadbad::acos(x); }
  static TU atan(const TU& x) { return fadbad::atan(x); }
  static TU erf (const TU& x) { throw std::runtime_error("mc::Op<fadbad::T<U>>::erf -- operation not permitted"); }
  static TU erfc(const TU& x) { throw std::runtime_error("mc::Op<fadbad::T<U>>::erfc -- operation not permitted"); }
  static TU fstep(const TU& x) { throw std::runtime_error("mc::Op<fadbad::T<U>>::fstep -- operation not permitted"); }
  static TU bstep(const TU& x) { throw std::runtime_error("mc::Op<fadbad::T<U>>::bstep -- operation not permitted"); }
  static TU hull(const TU& x, const TU& y) { throw std::runtime_error("mc::Op<fadbad::T<U>>::hull -- operation not permitted"); }
  static TU min (const TU& x, const TU& y) { throw std::runtime_error("mc::Op<fadbad::T<U>>::min -- operation not permitted"); }
  static TU max (const TU& x, const TU& y) { throw std::runtime_error("mc::Op<fadbad::T<U>>::max -- operation not permitted"); }
  static TU arh (const TU& x, const double k) { return fadbad::exp(-k/x); }
  static TU cheb(const TU& x, const unsigned n) { throw std::runtime_error("mc::Op<fadbad::B<U>>::max -- operation not permitted"); }
  template <typename X, typename Y> static TU pow(const X& x, const Y& y) { return fadbad::pow(x,y); }
  static TU monomial (const unsigned int n, const U* x, const int* k) { throw std::runtime_error("mc::Op<fadbad::T<U>>::monomial -- operation not permitted"); }
  static bool inter(TU& xIy, const TU& x, const TU& y) { xIy = x; return true; }
  static bool eq(const TU& x, const TU& y) { throw std::runtime_error("mc::Op<fadbad::T<U>>::eq -- operation not permitted"); }
  static bool ne(const TU& x, const TU& y) { throw std::runtime_error("mc::Op<fadbad::T<U>>::ne -- operation not permitted"); }
  static bool lt(const TU& x, const TU& y) { throw std::runtime_error("mc::Op<fadbad::T<U>>::lt -- operation not permitted"); }
  static bool le(const TU& x, const TU& y) { throw std::runtime_error("mc::Op<fadbad::T<U>>::le -- operation not permitted"); }
  static bool gt(const TU& x, const TU& y) { throw std::runtime_error("mc::Op<fadbad::T<U>>::gt -- operation not permitted"); }
  static bool ge(const TU& x, const TU& y) { throw std::runtime_error("mc::Op<fadbad::T<U>>::ge -- operation not permitted"); }
};

} // namespace mc

#endif
