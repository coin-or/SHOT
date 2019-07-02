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
  int M(5), N(4), KL(2), KU(1);
  
  CPPL::zgbmatrix A(M,N,KL,KU), B;
  for(int i=0; i<A.m; i++){ for(int j=0; j<A.n; j++){
    if(!( i-j>A.kl || j-i>A.ku )){
      A(i,j) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
    }
  }}
  
  cout << "A =\n" << A << endl;
  cout << "#### B.copy(A) ####" << endl;
  B.copy(A);
  cout << "B =\n" << B << endl;
  
  cout << "#### B.clear() ####" << endl;
  B.clear();
  cout << "B =\n" << B << endl;
  
  cout << "#### B.resize(4,4,1,2) & B.zero() ####" << endl;
  B.resize(4,4,1,2);
  B.zero();
  cout << "B =\n" << B << endl;
  
  cout << "#### B.identity() ####" << endl;
  B.identity();
  cout << "B =\n" << B << endl;
  
  return 0;
}

/*****************************************************************************/
