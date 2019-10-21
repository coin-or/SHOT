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
  int M(6), N(5), KL(2), KU(1);
  
  CPPL::dgbmatrix A;
  cout << "A ||"
       << " m=" << A.m << ", n=" << A.n
       << " kl=" << A.kl << ", ku=" << A.ku
       << ", array=" << A.array << endl;
  
  
  CPPL::dgbmatrix B(M,N,KL,KU);
  for(int i=0; i<B.m; i++){ for(int j=0; j<B.n; j++){
    if( !((i-j)>B.kl || (j-i)>B.ku) ){ B(i,j) =double( rand() /(RAND_MAX/10) ); }
  }}
  cout << "B ||"
       << " m=" << B.m << ", n=" << B.n
       << " kl=" << B.kl << ", ku=" << B.ku
       << ", array=" << B.array << endl;
  cout << "B =\n" << B << endl;
  
  
  CPPL::dgbmatrix C(B);
  cout << "C ||"
       << " m=" << C.m << ", n=" << C.n
       << " kl=" << C.kl << ", ku=" << C.ku
       << ", array=" << C.array << endl;
  cout << "C =\n" << C << endl;
  
  return 0;
}

/*****************************************************************************/
