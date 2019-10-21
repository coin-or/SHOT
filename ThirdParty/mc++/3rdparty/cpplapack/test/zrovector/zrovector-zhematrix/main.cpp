/*****************************************************************************/
/*                                 noname                                    */
/*****************************************************************************/

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
  
  CPPL::zrovector a(N), b(N);
  CPPL::zhematrix X(N), Y(N);
  for(int i=0; i<X.n; i++){ for(int j=0; j<=i; j++){
    X(i,j) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
    Y(i,j) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
  }}
  for(int i=0; i<a.l; i++){
    a(i) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
    b(i) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
  }
  CPPL::zhematrix Z(X+Y);
  
  cout << "a*Z-a*X-a*Y = (Should be zero)\n" << a*Z-a*X-a*Y << endl;
  
  cout << "a*Z-a*(X+Y) = (Should be zero)\n" << a*Z-a*(X+Y) << endl;
  
  cout << "(a+b)*X-a*X-b*X = (Should be zero)\n" << (a+b)*X-a*X-b*X << endl;
  
  cout << "(a-b)*(X-Y)-a*X+a*Y+b*X-b*Y = (Should be zero)\n"
       << (a-b)*(X-Y)-a*X+a*Y+b*X-b*Y << endl;
  
  return 0;
}

/*****************************************************************************/
