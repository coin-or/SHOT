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
  
  CPPL::zgbmatrix A(M,N,KL,KU);
  CPPL::zgematrix B(M,N), C(N,M);
  
  for(int i=0; i<A.m; i++){
    for(int j=std::max(0l,i-A.kl); j<std::min(A.n,i+A.ku+1); j++){
      A(i,j) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
    }
  }
  for(int i=0; i<B.m; i++){ for(int j=0; j<B.n; j++){
    B(i,j) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
  }}
  for(int i=0; i<C.m; i++){ for(int j=0; j<C.n; j++){
    C(i,j) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
  }}
  
  cout << "A =\n" << A << endl;
  cout << "B =\n" << B << endl;
  cout << "C =\n" << C << endl;
  
  cout << "A+B =\n" << A+B << endl;
  cout << "A-B =\n" << A-B << endl;
  cout << "A*C =\n" << A*C << endl;
  
  return 0;
}

/*****************************************************************************/
