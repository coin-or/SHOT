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
bool qmr
(
 const CPPL::dgematrix& A,
 CPPL::dcovector& x,
 const double& eps
)
{
  CPPL::dcovector r(x), d(x.l), s(x.l);
  CPPL::dcovector v, v2(r), w, w2(r), p(x.l), p2, q(x.l);
  double xi(nrm2(w2)), eta(-1.0), epsilon(1.0), delta(0.0),
    rho, rho_old(nrm2(v2)),  gamma, gamma_old(1.0), theta, theta_old(0.0),  beta;
  x.zero();
  p.zero();
  q.zero();
  d.zero();
  s.zero();

  int itc(0);
  const int itmax(2*x.l);
  do{
    std::cout << "itc=" << itc << ", fabs(damax(r))=" << fabs(damax(r)) << std::endl;

    v =v2/rho_old;
    w =w2/xi;
    delta =v%w;
    p =v -(xi*delta/epsilon)*p;
    q =w -(rho_old*delta/epsilon)*q;
    p2 =A*p;
    epsilon =q%p2;
    beta =epsilon/delta;
    v2 =p2-beta*v;
    rho =nrm2(v2);
    w2 =CPPL::t(t(q)*A)-beta*w;
    xi =nrm2(w2);
    theta =rho/(gamma_old*beta);
    gamma =1./sqrt(1.+theta*theta);
    eta = -eta*rho_old*gamma*gamma/(beta*gamma_old*gamma_old);
    d =eta*p +pow(theta_old*gamma, 2)*d;
    s =eta*p2 +pow(theta_old*gamma, 2)*s;

    x+=d;
    r-=s;
    gamma_old =gamma;
    rho_old =rho;
    theta_old =theta;
    itc++;
  }while(fabs(damax(r))>eps && itc<itmax);
  
  if(fabs(damax(r))<eps){ return 0; }
  else{ return 1; }
}
//=============================================================================
bool bicg
(
 const CPPL::dgematrix& A,
 CPPL::dcovector x,
 const double& eps
)
{
  double alpha, beta(0.0);
  const CPPL::dcovector y(x);
  CPPL::dcovector p_0(x.l), p_1, q_0(x.l), q_1, Ap1, Aq1;
  CPPL::dcovector r_1(y-A*x), r_2, s_1(r_1), s_2;
  //p_0.zero(); q_0.zero();
  
  int itc(0);
  const int itmax(2*x.l);
  while(fabs(damax(r_1))>eps && ++itc<itmax){
    std::cout << itc << " " << fabs(damax(y-A*x)) << std::endl;
    
    p_1 =r_1 +beta*p_0;
    q_1 =s_1 +beta*q_0;

    Ap1 =A*p_1;
    Aq1 =t(t(q_1)*A);
    alpha =(s_1%r_1)/(q_1%Ap1);

    x +=alpha*p_1;

    r_2 =r_1 -alpha*Ap1;
    s_2 =s_1 -alpha*Aq1;
    beta =(s_2%r_2)/(s_1%r_1);

    swap(p_0, p_1);  swap(q_0, q_1);
    swap(r_1, r_2);  swap(s_1, s_2);
  }
  std::cerr << "itc=" << itc << "  fabs(damax(r_1))=" << fabs(damax(r_1)) << std::endl;
  
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
  CPPL::dgematrix A(size,size);
  
  for(int i=0; i<size; i++){
    for(int j=0; j<size; j++){
      A(i,j) =(double(rand())/double(RAND_MAX))*2.0 -1.0;
    }
    A(i,i)+=10.;
  }
  A.write("A.dgematrix");
  
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
  if( qmr(A, y, eps) ){
    std::cerr << "failed." << std::endl;
    exit(1);
  }
  y.write("solution.dcovector");
  std::cout << "solution=\n" << t(y) << std::endl;
  
  return 0;
}
