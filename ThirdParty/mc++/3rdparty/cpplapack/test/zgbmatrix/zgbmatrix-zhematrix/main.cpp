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
  int M(4), N(4), KL(1), KU(2);
  
  CPPL::zgbmatrix A(M,N,KL,KU);
  CPPL::zhematrix X(N);
  
  for(int i=0; i<A.m; i++){ for(int j=0; j<A.n; j++){
    if(!( i-j>A.kl || j-i>A.ku )){
      A(i,j) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
    }
  }}
  
  for(int i=0; i<X.n; i++){
    for(int j=0; j<i; j++){
      X(i,j) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
    }
    X(i,i) =complex<double>(rand()/(RAND_MAX/10), 0.0);
  }
  cout << "A =\n" << A << endl;
  cout << "X =\n" << X << endl;
  
  cout << "A+X =\n" << A+X << endl;
  cout << "A-X =\n" << A-X << endl;
  cout << "A*X =\n" << A*X << endl;
  
  return 0;
}

/*****************************************************************************/
