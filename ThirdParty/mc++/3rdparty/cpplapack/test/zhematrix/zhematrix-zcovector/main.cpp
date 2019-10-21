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
  int N(3);
  
  CPPL::zhematrix A(N), B(N);
  CPPL::zcovector x(N), y(N);
  for(int i=0; i<A.n; i++){
    for(int j=0; j<i; j++){
      A(i,j) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
      B(i,j) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
    }
    A(i,i) =complex<double>(rand()/(RAND_MAX/10), 0.0);
    B(i,i) =complex<double>(rand()/(RAND_MAX/10), 0.0);
  }
  
  for(int i=0; i<x.l; i++){
    x(i) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
    y(i) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
  }
  
  CPPL::zcovector z(x+y);
  
  cout << "A =\n" << A << endl;
  cout << "B =\n" << B << endl;
  cout << "x =\n" << x << endl;
  cout << "y =\n" << y << endl;
  cout << "z =\n" << z << endl;

  cout << "A*z-A*x-A*y = (Should be zero)\n" << A*z-A*x-A*y << endl;
  cout << "A*z-A*(x+y) = (Should be zero)\n" << A*z-A*(x+y) << endl;
  cout << "(A+B)*x-A*x-B*x = (Should be zero)\n" << (A+B)*x-A*x-B*x << endl;
  cout << "(A-B)*(x-y)-A*x+A*y+B*x-B*y = (Should be zero)\n"
       << (A-B)*(x-y)-A*x+A*y+B*x-B*y << endl;
  
  return 0;
}

/*****************************************************************************/
