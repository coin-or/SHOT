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
  int M(5), N(5), KL(2), KU(2);
  
  CPPL::dsymatrix A(N);
  CPPL::dgbmatrix X(M,N,KL,KU), Y(M,N,KL,KU), Z(M,N,KL,KU);
  
  A.zero();
  for(int i=0; i<X.m; i++){
    for(int j=std::max(0l,i-X.kl); j<std::min(X.n,i+X.ku+1); j++){
      if(i>=j){
        A(i,j) = double( rand() /(RAND_MAX/10) );
        X(i,j) =X(j,i) =-A(i,j);
        Y(i,j) =Y(j,i) =+A(i,j);
      }
    }
  }
  
  for(int i=0; i<Z.m; i++){
    for(int j=std::max(0l,i-Z.kl); j<std::min(Z.n,i+Z.ku+1); j++){
      Z(i,j) = double( rand() /(RAND_MAX/10) );
    }
  }
  
  cout << "A =\n" << A << endl;
  cout << "X =\n" << X << endl;
  cout << "Y =\n" << Y << endl;
  cout << "Z =\n" << Z << endl;
  
  //dsy+dgb
  cout << "A+X = (Should be zero)\n" << A+X << endl;
  //dsy-dgb
  cout << "A-Y = (Should be zero)\n" << A-Y << endl;
  //dsy*dgb, t(_dge), t(dgb), _dgb*dsy, _dge-_dge
  cout << "t(A*Z)-t(Z)*A = (Should be zero)\n" << t(A*Z)-t(Z)*A << endl;

  //dsy+_dgb, -dgb
  cout << "A+(-Y) = (Should be zero)\n" << A+(-Y) << endl;
  //dsy-_dgb, -dgb
  cout << "A-(-X) = (Should be zero)\n" << A-(-X) << endl;
  //dsy*_dgb, dgb+dgb, dsy*dgb, _dge+_dge, _dge-_dge
  cout << "A*(X+Z) - (A*X+A*Z) = (Should be zero)\n"
       << A*(X+Z) - (A*X+A*Z) << endl;
  
  //_dsy+dgb, -dsy
  cout << "(-A)+Y = (Should be zero)\n" << (-A)+Y << endl;

  //_dsy-dgb, -dsy
  cout << "(-A)-X = (Should be zero)\n" << (-A)-X << endl;
  //_dsy*dgb, -dsy, dsy*dgb, _dge+_dge
  cout << "(-A)*Z+(A*Z) = (Should be zero)\n" << ((-A)*Z+(A*Z)) << endl;
  
  //_dsy+_dgb, -dsy, -dgb
  cout << "(-A)+(-X) = (Should be zero)\n" << (-A)+(-X) << endl;
  //_dsy-_dgb, -dsy, -dgb
  cout << "(-A)-(-Y) = (Should be zero)\n" << (-A)-(-Y) << endl;
  //_dsy*_dgb, -dsy, -dgb, dsy*dgb, _dge-_dge
  cout << "(-A)*(-Z)-(A*Z) = (Should be zero)\n" << (-A)*(-Z)-(A*Z) << endl;
  
  return 0;
}

/*****************************************************************************/
