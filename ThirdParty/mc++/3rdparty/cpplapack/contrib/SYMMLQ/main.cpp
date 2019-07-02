/*****************************************************************************/
/*                                main.cpp                                   */
/*****************************************************************************/
//=============================================================================
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <vector>
#include "cpplapack.h"

//=============================================================================
bool symmlq
(
 const CPPL::dsymatrix& A,
 CPPL::dcovector& x,
 const double& eps_r
)
{
  const CPPL::dcovector y(x);
  const double eps(1e-16);
  const double shift(0.);

  double g_bar, alpha, b_old, beta, beta1, z, z_bar, s, t, rhs1, rhs2, d_bar, bstep, snprod;
  x.zero();
  CPPL::dcovector w(x), v(x), X(x), Y1(y), Y2(y), Y(y);

  beta1 =nrm2(y);
  s =1./beta1;
  v =s*y;

  Y =A*v -shift*v;
  alpha =v%Y;
  Y -=(alpha/beta1)*Y1;
  //std::cerr << "Y=" << CPPL::t(Y) << std::endl;

  z =v%Y;
  s =v%v;
  Y -=(z/s)*v;
  //std::cerr << "Y=" << CPPL::t(Y) << std::endl;

  Y2 =Y;
  b_old =beta1;
  beta =sqrt(Y2%Y);

  double denom;
  denom =sqrt(s)*nrm2(Y2) +eps;
  s =z/denom;
  t =(v%Y2)/denom;

  rhs1 =beta1;
  rhs2 =0.;
  g_bar =alpha;
  d_bar =beta;
  bstep =0.;
  snprod =1.;
  
  int itc(0);
  const int itmax(2*x.l);
  while( fabs(damax(y-A*x))>eps_r && itc<itmax ){
    std::cout << "# "<< itc << " " << fabs(damax(y-A*x)) << std::endl;
    z_bar =rhs1/g_bar;
    z =(snprod*z_bar+bstep)/beta1;

    s =1./beta;
    v =s*Y;
    Y =A*v -(beta/b_old)*Y1 -shift*v;
    alpha =v%Y;
    Y -=(alpha/beta)*Y2;
    Y1 =Y2;
    Y2 =Y;

    b_old =beta;
    beta =sqrt(Y2%Y);

    double gamma, cs, sn, delta, epsilon;
    gamma =sqrt(pow(g_bar,2)+pow(b_old,2));
    cs =g_bar/gamma;
    sn =b_old/gamma;
    delta =cs*d_bar +sn*alpha;
    g_bar =sn*d_bar -cs*alpha;
    epsilon =sn*beta;
    d_bar =-cs*beta;

    z =rhs1/gamma;
    s =z*cs;
    t =z*sn;
    X +=s*w +t*v;
    w =sn*w -cs*v;

    bstep +=snprod*cs*z;
    snprod *=sn;
    rhs1 =rhs2 -delta*z;
    rhs2 =-epsilon*z;

    x =X +(bstep/beta1)*y;
    //std::cout << "x=" << CPPL::t(x) << std::endl;
    itc++;
  }
  std::cerr << "itc=" << itc << "  fabs(damax(y-A*x))=" << fabs(damax(y-A*x)) << std::endl;
  //std::cerr << "itc=" << itc << "  fabs(f)=" << fabs(f) << std::endl;

  if(itc<itmax){ return 0; }
  else{ return 1; }
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
int main(int argc, char** argv)
{
  srand(time(NULL));
  
  const int size(100);
  CPPL::dsymatrix A(size);
  
  for(int i=0; i<size; i++){
    for(int j=0; j<=i; j++){
      A(i,j) =(double(rand())/double(RAND_MAX))*2.0 -1.0;
    }
    A(i,i)+=10.;
  }
  A.write("A.dsymatrix");
  
  CPPL::dcovector x(size);
  for(int i=0; i<size; i++){
    x(i) =(double(rand())/double(RAND_MAX))*1. -0.5;
  }
  x.write("answer.dcovector");//solution
  std::cerr << "answer=\n" << t(x) << std::endl;
  
  CPPL::dcovector y(size);
  y=A*x;
  y.write("y.dcovector");
  
  double eps(fabs(damax(y))*1e-6);
  //std::cerr << "eps=" << eps << std::endl;
  if( symmlq(A, y, eps) ){
    std::cerr << "failed." << std::endl;
    exit(1);
  }
  y.write("solution.dcovector");
  std::cout << "solution=\n" << t(y) << std::endl;
  
  return 0;
}
