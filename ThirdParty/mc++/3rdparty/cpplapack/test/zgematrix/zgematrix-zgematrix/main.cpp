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
  int M(4), N(3), K(2);
  
  CPPL::zgematrix A(M,K), B(K,N), C;
  for(int i=0; i<A.m; i++){ for(int j=0; j<A.n; j++){
    A(i,j) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
  }}
  for(int i=0; i<B.m; i++){ for(int j=0; j<B.n; j++){
    B(i,j) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
  }}
  
  cout << "A =\n" << A << endl;
  cout << "B =\n" << B << endl;
  
  cout << "A+A =\n" << A+A << endl;
  cout << "A-A =\n" << A-A << endl;
  cout << "A*B =\n" << A*B << endl;
  
  cout << "#### C=A; ####" << endl;
  C=A;
  cout << "C =\n" << C << endl;
  cout << "#### C+=A; ####" << endl;
  C+=A;
  cout << "C =\n" << C << endl;
  cout << "#### C-=A; ####" << endl;
  C-=A;
  cout << "C =\n" << C << endl;
  cout << "#### C*=B; ####" << endl;
  C*=B;
  cout << "C =\n" << C << endl;
  cout << "#### C=A+A; ####" << endl;
  C=A+A;
  cout << "C =\n" << C << endl;
  
  return 0;
}

/*****************************************************************************/
