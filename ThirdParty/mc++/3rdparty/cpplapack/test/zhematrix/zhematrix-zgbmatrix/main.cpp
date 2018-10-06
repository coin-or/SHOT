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
  int M(3), N(3), KL(1), KU(2);
  
  CPPL::zhematrix A(N);
  CPPL::zgbmatrix B(M,N,KL,KU);
  
  for(int i=0; i<A.n; i++){
    for(int j=0; j<i; j++){
      A(i,j) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
    }
    A(i,i) =complex<double>(rand()/(RAND_MAX/10), 0.0);
  }
  
  for(int i=0; i<B.m; i++){
    for(int j=std::max(0l,i-B.kl); j<std::min(B.n,i+B.ku+1); j++){
      B(i,j) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
    }
  }
  
  
  cout << "A =\n" << A << endl;
  cout << "B =\n" << B << endl;
  
  cout << "A+B =\n" << A+B << endl;
  cout << "A-B =\n" << A-B << endl;
  cout << "A*B =\n" << A*B << endl;
  
  return 0;
}

/*****************************************************************************/
