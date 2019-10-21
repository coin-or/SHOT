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
  int M(4), N(3);
  
  //// make dgematrix A ////
  CPPL::dgematrix A(M,N);
  for(int i=0; i<A.m; i++){ for(int j=0; j<A.n; j++){
    A(i,j) =double( rand() /(RAND_MAX/10) );
  }}
  
  //// print A in two ways ////
  std::cout << "A =\n" << A << std::endl;
  for(int i=0; i<A.m; i++){ for(int j=0; j<A.n; j++){
    std::cout << "A(" << i << "," << j << ") =" << A(i,j) << std::endl;
  }}
  
  //// make A 10 times ////
  std::cout << "#### A*=10.; ####" << std::endl;
  A*=10.;
  
  //// make dgematrix B ////
  CPPL::dgematrix B(A);
  std::cout << "B =\n" << B << std::endl;
  for(int i=0; i<B.m; i++){ for(int j=0; j<B.n; j++){
    std::cout << "B(" << i << "," << j << ") =" << B(i,j) << std::endl;
  }}
  
  //// print A+B ////
  std::cout << "A+B=\n" << A+B << std::endl;
  
  //// write/read ////
  std::cout << "writeing B into tmp.txt" << std::endl;
  B.write( "tmp.txt" );
  CPPL::dgematrix C;
  std::cout << "reading tmp.txt into C" << std::endl;
  C.read( "tmp.txt" );
  std::cout << "C-B =\n" << C-B << "<-Should be zero." << std::endl;
  
  
  //// const ////
  const CPPL::dgematrix X( CPPL::dgematrix(2,2)
                           .set(0,0,1).set(0,1,2).set(1,0,3).set(1,1,4) );
  std::cout << "X=\n" << X << std::endl;
  
  return 0;
}
