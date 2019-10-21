/*****************************************************************************/
/*                               noname.cpp                                  */
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
  CPPL::dgematrix A(M,N);
  CPPL::dgbmatrix B(M,N,KL,KU);
  CPPL::dsymatrix C(N);
  CPPL::dcovector cv(M);
  CPPL::drovector rv(M);
  
  for(int i=0; i<M; i++){ for(int j=0; j<N; j++){
    A(i,j) =double( rand() /(RAND_MAX/10) );
  }}
  
  for(int i=0; i<M; i++){ for(int j=0; j<N; j++){
    if( !(i-j>B.kl || j-i>B.ku) ){
      B(i,j) =double( rand() /(RAND_MAX/10) );
    }
  }}
  
  for(int i=0; i<N; i++){ for(int j=0; j<=i; j++){
    C(i,j) =double( rand() /(RAND_MAX/10) );
  }}
  
  for(int i=0; i<M; i++){
    cv(i) =double( rand() /(RAND_MAX/10) );
    rv(i) =double( rand() /(RAND_MAX/10) );
  }
  
  cout << "A =\n" << A << endl;
  cout << "10.*A =\n" << 10.*A << endl;
  cout << "A*10. =\n" << A*10. << endl;
  cout << "A/10. =\n" << A/10. << endl;
  
  cout << "B =\n" << B << endl;
  cout << "10.*B =\n" << 10.*B << endl;
  cout << "B*10. =\n" << B*10. << endl;
  cout << "B/10. =\n" << B/10. << endl;

  cout << "C =\n" << C << endl;
  cout << "10.*C =\n" << 10.*C << endl;
  cout << "C*10. =\n" << C*10. << endl;
  cout << "C/10. =\n" << C/10. << endl;
  
  cout << "cv =\n" << cv << endl;
  cout << "10.*cv =\n" << 10.*cv << endl;
  cout << "cv*10. =\n" << cv*10. << endl;
  cout << "cv/10. =\n" << cv/10. << endl;
  
  cout << "rv =\n" << rv << endl;
  cout << "10.*rv =\n" << 10.*rv << endl;
  cout << "rv*10. =\n" << rv*10. << endl;
  cout << "rv/10. =\n" << rv/10. << endl;
  
  return 0;
}

/*****************************************************************************/
