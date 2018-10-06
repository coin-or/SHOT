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
  int N(3), M(2);
  
  CPPL::zrovector x(M), y;
  CPPL::zgematrix A(M,N);
  for(int i=0; i<x.l; i++){
    x(i) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
  }
  for(int i=0; i<A.m; i++){ for(int j=0; j<A.n; j++){
    A(i,j) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
  }}
  
  cout << "x =\n" << x << endl;
  cout << "A =\n" << A << endl;
  
  cout << "#### y=x*A; ####" << endl;
  y=x*A;
  cout << "y =\n" << y << endl;
  
  return 0;
}

/*****************************************************************************/
