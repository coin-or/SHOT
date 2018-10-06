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
  int M(3), N(4);
  
  CPPL::zgematrix A(M,N);
  for(int i=0; i<A.m; i++){ for(int j=0; j<A.n; j++){
    A(i,j) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
  }}
  
  cout << "A =\n" << A << endl;
  
  cout << "#### t(A) ####" << endl;
  cout << "t(A) =\n" << CPPL::t(A) << endl;
  
  cout << "#### i(A) ####" << endl;
  A.resize(M,M);
  for(int i=0; i<A.m; i++){ for(int j=0; j<A.n; j++){
    A(i,j) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
  }}
  CPPL::zgematrix A_inv;
  A_inv =CPPL::i(A);
  //A_inv =i(A); // g++ cannot compile this line
  cout << "A =\n" << A << endl;
  cout << "A_inv =\n" << A_inv << endl;
  cout << "A*A_inv =\n" << A*A_inv << endl;
  
  //// max ////
  cout << "A =\n" << A << endl;
  cout << "damax(A) =\n" << damax(A) << endl;
  cout << "#### idamax(p,q, A) ####" << endl;
  long p,q;
  idamax(p,q, A);
  cout << "p=" << p << ", q=" << q << endl;
  
  return 0;
}

/*****************************************************************************/
