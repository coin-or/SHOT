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
  int M(4), N(3), KL(2), KU(1);
  
  CPPL::drovector x(M), y;
  CPPL::dgbmatrix A(M,N,KL,KU);
  for(int i=0; i<x.l; i++){
    x(i) =double( rand() /(RAND_MAX/10) );
  }
  for(int i=0; i<A.m; i++){
    for(int j=std::max(0l,i-A.kl); j<std::min(A.n,i+A.ku+1); j++){
      A(i,j) =double( rand() /(RAND_MAX/10) );
    }
  }
  
  cout << "x =\n" << x << endl;
  cout << "A =\n" << A << endl;
  
  cout << "#### y=x*A; ####" << endl;
  y=x*A;
  cout << "y =\n" << y << endl;
  
  return 0;
}

/*****************************************************************************/
