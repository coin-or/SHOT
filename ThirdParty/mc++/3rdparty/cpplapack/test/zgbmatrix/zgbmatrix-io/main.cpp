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
  int M(5), N(4), KL(3), KU(2);
  
  CPPL::zgbmatrix A(M,N,KL,KU);
  for(int i=0; i<A.m; i++){ for(int j=0; j<A.n; j++){
    if(!( i-j>A.kl || j-i>A.ku )){
      A(i,j) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
    }
  }}
  
  cout << "A =\n" << A << endl;
  for(int i=0; i<A.m; i++){ for(int j=0; j<A.n; j++){
    if(!( i-j>A.kl || j-i>A.ku )){
      cout << "A(" << i << "," << j << ") =" << A(i,j) << endl;
    }
  }}
  
  
  
  const CPPL::zgbmatrix B(A);
  cout << "B =\n" << B << endl;
  
  //B*=10.; //compile error
  //B(0,0)=0.; //compile error
  
  //cout << "A+B=\n" << A+B << endl;
  
  //// write/read ////
  B.write( "tmp.txt" );
  CPPL::zgbmatrix C;
  C.read( "tmp.txt" );
  cout << "C-B =\n" << C-B << "<-Should be zero." << endl;

  return 0;
}

/*****************************************************************************/
