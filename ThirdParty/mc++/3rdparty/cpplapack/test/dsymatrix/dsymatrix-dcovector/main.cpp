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
  
  CPPL::dsymatrix A(N), B(N);
  CPPL::dcovector x(N), y(N);
  for(int i=0; i<A.n; i++){ for(int j=0; j<=i; j++){
    A(i,j) =double( rand() /(RAND_MAX/10) );
    B(i,j) =double( rand() /(RAND_MAX/10) );
  }}
  for(int i=0; i<x.l; i++){
    x(i) =double( rand() /(RAND_MAX/10) );
    y(i) =double( rand() /(RAND_MAX/10) );
  }
  CPPL::dcovector z(x+y);
  
  cout << "A*z-A*x-A*y =\n" << A*z-A*x-A*y << "<-Should be zero." << endl;
  cout << "A*z-A*(x+y) =\n" << A*z-A*(x+y) << "<-Should be zero." << endl;
  cout << "(A+B)*x-A*x-B*x =\n" << (A+B)*x-A*x-B*x
       << "<-Should be zero." << endl;
  cout << "(A-B)*(x-y)-A*x+A*y+B*x-B*y =\n" << (A-B)*(x-y)-A*x+A*y+B*x-B*y
       << "<-Should be zero." << endl;
  
  return 0;
}

/*****************************************************************************/
