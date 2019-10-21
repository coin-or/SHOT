/*****************************************************************************/
/*                               noname.cpp                                  */
/*****************************************************************************/

//====================================================================[include]
#include <iostream>
#include <cstdlib>
#include <ctime>
#include "cpplapack.h"

//=======================================================================[main]
/*! main */
int main(int argc, char** argv)
{
  srand(time(NULL));
  int M(4), N(3), KL(2), KU(1);
  CPPL::zgematrix A(M,N);
  CPPL::zgbmatrix B(M,N,KL,KU);
  CPPL::zhematrix C(N);
  CPPL::zcovector cv(M);
  CPPL::zrovector rv(M);
  
  for(int i=0; i<M; i++){ for(int j=0; j<N; j++){
    A(i,j) =std::complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
  }}
  
  for(int i=0; i<M; i++){ for(int j=0; j<N; j++){
    if( !(i-j>B.kl || j-i>B.ku) ){
      B(i,j) =std::complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
    }
  }}
  
  for(int i=0; i<N; i++){
    for(int j=0; j<i; j++){
      C(i,j) =std::complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
    }
    C(i,i) =std::complex<double>(rand()/(RAND_MAX/10), 0.0);
  }
  
  for(int i=0; i<M; i++){
    cv(i) =std::complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
    rv(i) =std::complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
  }
  
  std::cout << "A =\n" << A << std::endl;
  std::cout << "(10.,10.)*A =\n" << std::complex<double>(10.,10.)*A << std::endl;
  std::cout << "A*(10.,10.) =\n" << A*std::complex<double>(10.,10.) << std::endl;
  std::cout << "A/(10.,10.) =\n" << A/std::complex<double>(10.,10.) << std::endl;
  
  std::cout << "B =\n" << B << std::endl;
  std::cout << "(10.,10.)*B =\n" << std::complex<double>(10.,10.)*B << std::endl;
  std::cout << "B*(10.,10.) =\n" << B*std::complex<double>(10.,10.) << std::endl;
  std::cout << "B/(10.,10.) =\n" << B/std::complex<double>(10.,10.) << std::endl;

  std::cout << "C =\n" << C << std::endl;
  std::cout << "(10.,10.)*C =\n" << std::complex<double>(10.,10.)*C << std::endl;
  std::cout << "C*(10.,10.) =\n" << C*std::complex<double>(10.,10.) << std::endl;
  std::cout << "C/(10.,10.) =\n" << C/std::complex<double>(10.,10.) << std::endl;
  
  std::cout << "cv =\n" << cv << std::endl;
  std::cout << "(10.,10.)*cv =\n" << std::complex<double>(10.,10.)*cv << std::endl;
  std::cout << "cv*(10.,10.) =\n" << cv*std::complex<double>(10.,10.) << std::endl;
  std::cout << "cv/(10.,10.) =\n" << cv/std::complex<double>(10.,10.) << std::endl;
  
  std::cout << "rv =\n" << rv << std::endl;
  std::cout << "(10.,10.)*rv =\n" << std::complex<double>(10.,10.)*rv << std::endl;
  std::cout << "rv*(10.,10.) =\n" << rv*std::complex<double>(10.,10.) << std::endl;
  std::cout << "rv/(10.,10.) =\n" << rv/std::complex<double>(10.,10.) << std::endl;
  
  return 0;
}

/*****************************************************************************/
