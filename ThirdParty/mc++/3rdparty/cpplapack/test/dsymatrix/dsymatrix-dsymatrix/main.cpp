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
  
  CPPL::dsymatrix A(N), X(N), Y(N), Z(N);
  for(int i=0; i<A.n; i++){ for(int j=0; j<=i; j++){
    A(i,j) =double( rand() /(RAND_MAX/10) );
    X(i,j) = -A(i,j);
    Y(i,j) = A(i,j);
    Z(i,j) =double( rand() /(RAND_MAX/10) );
  }}
  
    
  cout << "A =\n" << A << endl;
  cout << "X =\n" << X << endl;
  cout << "Y =\n" << Y << endl;
  cout << "Z =\n" << Z << endl;

  //dsy+dsy
  cout << "A+X =\n" << A+X << "<-Should be zero." << endl;
  //dsy-dsy
  cout << "A-Y =\n" << A-Y << "<-Should be zero." << endl;
  //dsy*dsy
  cout << "t(A*Z)-Z*A =\n" << t(A*Z)-Z*A << "<-Should be zero." << endl;

  //dsy=dsy, dsy-dsy
  CPPL::dsymatrix B;
  cout << "(B=A)-A =\n" << (B=A)-A << "<-Should be zero." << endl;  
  //dsy+=dsy
  cout << "(B+=Z) =\n" << (B+=Z) << endl;  
  //dsy-=dsy
  cout << "(B-=A) =\n" << (B-=A) << endl;  
  
  //dsy+_dsy, -dsy
  cout << "A+(-Y) =\n" << A+(-Y) << "<-Should be zero." << endl;
  //dsy-_dsy, -dsy
  cout << "A-(-X) =\n" << A-(-X) << "<-Should be zero." << endl;
  //dsy*_dsy, dsy+dsy, dsy*dsy, _dge+_dge, _dge-_dge
  cout << "A*(X+Z) - (A*X+A*Z) =\n" << A*(X+Z) - (A*X+A*Z) << "<-Should be zero." << endl;
  
  //dsy=_dsy
  cout << "(B=-A)+A =\n" << (B=-A)+A << "<-Should be zero." << endl;  
  //dsy+=_dsy
  cout << "(B+=-Z) =\n" << (B+=-Z) << endl;  
  //dsy-=_dsy
  cout << "(B-=-A) =\n" << (B-=-A) << endl;  
  
  //_dsy+dsy, -dsy
  cout << "(-A)+Y =\n" << (-A)+Y << "<-Should be zero." << endl;
  //_dsy-dsy, -dsy
  cout << "(-A)-X =\n" << (-A)-X << "<-Should be zero." << endl;
  //_dsy*dsy, -dsy, dsy*dsy, _dge+_dge
  cout << "(-A)*Z+(A*Z) =\n" << ((-A)*Z+(A*Z)) << "<-Should be zero." << endl;
  
  //_dsy+_dsy, -dsy, -dsy
  cout << "(-A)+(-X) =\n" << (-A)+(-X) << "<-Should be zero." << endl;
  //_dsy-_dsy, -dsy, -dsy
  cout << "(-A)-(-Y) =\n" << (-A)-(-Y) << "<-Should be zero." << endl;
  //_dsy*_dsy, -dsy, -dsy, dsy*dsy, _dge-_dge
  cout << "(-A)*(-Z)-(A*Z) =\n" << (-A)*(-Z)-(A*Z) << "<-Should be zero." << endl;
  
  return 0;
}

/*****************************************************************************/
