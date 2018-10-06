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
  int N(3);
  
  CPPL::zhematrix A(N), B(N);
  CPPL::zgematrix X(N,N), Y(N,N), Z(N,N);
  for(int i=0; i<A.n; i++){
    for(int j=0; j<i; j++){
      A(i,j) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
      B(i,j) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
    }
    A(i,i) =complex<double>(rand()/(RAND_MAX/10), 0.0);
    B(i,i) =complex<double>(rand()/(RAND_MAX/10), 0.0);
  }
  
  for(int i=0; i<A.n; i++){ for(int j=0; j<A.n; j++){
    if(i>=j){
      X(i,j) = -A(i,j);
      Y(i,j) = A(i,j);
    }
    else{
      X(i,j) = conj(-A(j,i));
      Y(i,j) = conj(A(j,i));
    }      
  }}
  
  for(int i=0; i<A.n; i++){ for(int j=0; j<A.n; j++){
    Z(i,j) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
  }}
  
  cout << "A =\n" << A << endl;
  cout << "B =\n" << B << endl;
  cout << "X =\n" << X << endl;
  cout << "Y =\n" << Y << endl;
  cout << "Z =\n" << Z << endl;
  
  cout << "X+A = (Should be zero)\n" << X+A << endl;
  cout << "Y-A = (Should be zero)\n" << Y-A << endl;
  cout << "Z*A-Z*Y = (Should be zero)\n" << Z*A-Z*Y << endl;

  CPPL::zgematrix W;
  cout << "W=-A;" << endl;
  W=-A.to_zgematrix();
  cout << "W =\n" << W << endl;
  cout << "W+=A;" << endl;
  W+=A;
  cout << "W = (Should be zero)\n" << W << endl;
  cout << "W=A;" << endl;
  W=A.to_zgematrix();
  cout << "W-=A;" << endl;
  W-=A;
  cout << "W = (Should be zero)\n" << W << endl;
  cout << "W.identity();" << endl;
  W.identity();
  cout << "W*=A;" << endl;
  W*=A;
  cout << "W =\n" << W << endl;
  
  return 0;
}

/*****************************************************************************/
