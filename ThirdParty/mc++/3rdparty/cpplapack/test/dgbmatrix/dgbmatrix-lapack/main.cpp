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
  int M(6), N(6), KL(1), KU(1);
  
  CPPL::dgbmatrix A(M,N,KL,KU);
  for(int i=0; i<A.m; i++){
    for(int j=std::max(0l,i-A.kl); j<std::min(A.n,i+A.ku+1); j++){
      A(i,j) =double( rand() /(RAND_MAX/10) );
    }
  }
  
  A.resize(M,M,KL,KU);
  for(int i=0; i<A.m; i++){
    for(int j=std::max(0l,i-A.kl); j<std::min(A.n,i+A.ku+1); j++){
      A(i,j) =double( rand() /(RAND_MAX/10) );
    }
  }
  CPPL::dgbmatrix A_orig(A);
  
  CPPL::dcovector y(M);
  for(int i=0; i<y.l; i++){
    y(i) =double( rand() /(RAND_MAX/10) );
  }
  
  cout << "A =\n" << A << endl;
  cout << "y =\n" << y << endl;
  cout << "#### A.dgbsv(y) ####" << endl;
  A.dgbsv(y);
  cout << "A =\n" << A << endl;
  cout << "y =\n" << y << endl;
  cout << "A_orig*y =\n" << A_orig*y << endl;
  
  
  cout << "#### A=A_orig ####" << endl;
  A=A_orig;
  
  CPPL::dgematrix Y(M,N);
  for(int i=0; i<Y.m; i++){ for(int j=0; j<Y.n; j++){
    Y(i,j) =double( rand() /(RAND_MAX/10) );
  }}
  cout << "A =\n" << A << endl;
  cout << "Y =\n" << Y << endl;
  cout << "#### A.dgbsv(Y) ####" << endl;
  A.dgbsv(Y);
  cout << "A =\n" << A << endl;
  cout << "Y =\n" << Y << endl;
  cout << "A_orig*Y =\n" << A_orig*Y << endl;
  
  return 0;
}

/*****************************************************************************/
