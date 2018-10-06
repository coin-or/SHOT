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
  int N(3);
  
  //// make dgematrix A ////
  CPPL::zhematrix A(N);
  for(int i=0; i<A.n; i++){
    for(int j=0; j<i; j++){
      A(i,j) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
    }
    A(i,i) =complex<double>(rand()/(RAND_MAX/10), 0.0);
  }
  
  //// print A in two ways ////
  cout << "A =\n" << A << endl;
  for(int i=0; i<A.n; i++){ for(int j=0; j<=i; j++){
    cout << "A(" << i << "," << j << ") =" << A(i,j) << endl;
  }}
  
  for(int i=0; i<A.n; i++){ 
    for(int j=0; j<A.n; j++){
      cout << A.array[i+A.n*j] << ' ';
    }
    cout << endl;
  }
  
  //// make A 10 times ////
  cout << "#### A*=10.; ####" << endl;
  A*=10.;
  
  //// make zgematrix B ////
  CPPL::zhematrix B(A);
  cout << "B =\n" << B << endl;
  for(int i=0; i<B.n; i++){ for(int j=0; j<=i; j++){
    cout << "B(" << i << "," << j << ") =" << B(i,j) << endl;
  }}
  
  //// print A+B ////
  cout << "A+B=\n" << A+B << endl;
  
  //// write/read ////
  B.write( "tmp.txt" );
  CPPL::zhematrix C;
  C.read( "tmp.txt" );
  cout << "C-B =\n" << C-B << "<-Should be zero." << endl;
  
  return 0;
}
