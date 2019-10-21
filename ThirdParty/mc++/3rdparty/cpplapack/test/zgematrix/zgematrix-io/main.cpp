//====================================================================[include]
#include <iostream>
#include <cstdlib>
#include <ctime>
#include "cpplapack.h"
using namespace std;

//=======================================================================[main]
/*! main */
int main(int argc, char** argv)
{
  srand(time(NULL));
  int M(4), N(3);
  
  //// make zgematrix A ////
  CPPL::zgematrix A(M,N);
  for(int i=0; i<A.m; i++){ for(int j=0; j<A.n; j++){
    A(i,j) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
  }}
  
  //// print A in two ways ////
  cout << "A =\n" << A << endl;
  for(int i=0; i<A.m; i++){ for(int j=0; j<A.n; j++){
    cout << "A(" << i << "," << j << ") =" << A(i,j) << endl;
  }}
  
  //// make A 10 times ////
  cout << "#### A*=10.; ####" << endl;
  A*=10.;
  
  //// make zgematrix B ////
  CPPL::zgematrix B(A);
  cout << "B =\n" << B << endl;
  for(int i=0; i<B.m; i++){ for(int j=0; j<B.n; j++){
    cout << "B(" << i << "," << j << ") =" << B(i,j) << endl;
  }}
  
  //// print A+B ////
  cout << "A+B=\n" << A+B << endl;
  
  //// write/read ////
  B.write( "tmp.txt" );
  CPPL::zgematrix C;
  C.read( "tmp.txt" );
  cout << "C-B =\n" << C-B << "<-Should be zero." << endl;
  
  return 0;
}
