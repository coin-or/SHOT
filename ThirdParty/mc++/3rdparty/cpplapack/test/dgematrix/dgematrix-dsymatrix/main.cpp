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
  int M(4), N(3);
  
  CPPL::dsymatrix A(N), B(N);
  CPPL::dgematrix X(N,N), Y(N,N), Z(N,N), G(M,N);
  for(int i=0; i<A.n; i++){
    for(int j=0; j<A.n; j++){
      A(i,j) =double( rand() /(RAND_MAX/10) );
      B(i,j) =double( rand() /(RAND_MAX/10) );
      Z(i,j) =double( rand() /(RAND_MAX/10) );
    }
  }
  for(int i=0; i<G.m; i++){
    for(int j=0; j<G.n; j++){
      G(i,j) = double( rand() /(RAND_MAX/10) );
    }
  }
  for(int i=0; i<A.n; i++){
    for(int j=0; j<A.n; j++){
      X(i,j) = -A(i,j);
      Y(i,j) = A(i,j);
    }
  }
  
  
  cout << "A =\n" << A << endl;
  cout << "B =\n" << B << endl;
  cout << "X =\n" << X << endl;
  cout << "Y =\n" << Y << endl;
  cout << "Z =\n" << Z << endl;
  cout << "G =\n" << G << endl;

  
  //dge+dsy
  cout << "X+A =\n" << X+A << "<-Should be zero." << endl;
  //dge-dsy
  cout << "Y-A =\n" << Y-A << "<-Should be zero." << endl;
  //dge*dsy, t(_dge), t(dge), dsy*_dge, _dge-_dge
  cout << "t(Y*A)-A*t(Y) =\n" << t(Y*A)-A*t(Y) << "<-Should be zero." << endl;
  cout << "G*A-G*Y =\n" << G*A-G*Y << "<-Should be zero." << endl;

  //dge/dsy
  //N/A
  
  //dge-dsy
  CPPL::dgematrix W;
  W =A.to_dgematrix();
  cout << "W-A =\n" << W-A << "<-Should be zero." << endl;
  
  //dge+=dsy
  cout << "W+=A =\n" << (W+=A) << endl;
  //dge-=dsy
  cout << "W-=B =\n" << (W-=B) << endl;
  //dge*=dsy, double*dsy, _dsy-dsy, _dsy*dsy, dge-_dsy
  cout << "(W*=B)-((2*A-B)*B) =\n" << (W*=B)-((2*A-B)*B) << "<-Should be zero" << endl;
  //N/A
  //dge/=dsy
  //N/A
  
  //dge+_dsy, -dsy
  cout << "Y+(-A) =\n" << Y+(-A) << "<-Should be zero." << endl;
  //dge-_dsy, -dsy
  cout << "X-(-A) =\n" << X-(-A) << "<-Should be zero." << endl;
  //dge*_dsy, dge*dsy, _dge-_dge
  cout << "X*(A+B) - X*A - X*B =\n" << X*(A+B) - X*A - X*B << "<-Should be zero." << endl;
  //dge/_dsy
  //N/A
  //dge+=_dsy, -dsy
  cout << "W+=-A =\n" << (W+=-A) << endl;
  //dge-=_dsy, dsy+dsy
  cout << "W-=(A+B) =\n" << (W-=(A+B)) << endl;
  //dge*=_dsy, dsy*dsy, dge+_dge
  cout << "(W*=B)+A*B =\n" << (W*=B)+A*B << "<-Should be zero" << endl;
  //dge/=_dsy
  //N/A
  
  //_dge+dsy
  cout << "(-Y)+A =\n" << (-Y)+A << "<-Should be zero." << endl;
  //_dge-dsy, -dge
  cout << "(-X)-A =\n" << (-X)-A << "<-Should be zero." << endl;
  //_dge*dsy, -dge, dge*dsy, _dge+_dge
  cout << "(-Z)*A+(Z*A) =\n" << (-Z)*A+(Z*A) << "<-Should be zero." << endl;
  //_dge/dsy
  //N/A
  //_dge=dsy
  //N/A
  //_dge+=dsy
  //N/A
  //_dge-=dsy
  //N/A
  //_dge*=dsy
  //N/A
  //_dge/=dsy
  //N/A

  //_dge+_dsy, -dge, -dsy
  cout << "(-X)+(-A) =\n" << (-X)+(-A) << "<-Should be zero." << endl;
  //_dge-_dsy, -dge, -dsy
  cout << "(-Y)-(-A) =\n" << (-Y)-(-A) << "<-Should be zero." << endl;
  //_dge*_dsy, -dge, -dsy, dge*dsy, _dge-_dge
  cout << "(-Z)*(-A)-(Z*A) =\n" << (-Z)*(-A)-(Z*A) << "<-Should be zero." << endl;
  //_dge/_dsy
  //N/A
  //_dge=_dsy
  //N/A
  //_dge+=_dsy
  //N/A
  //_dge-=_dsy
  //N/A
  //_dge*=_dsy
  //N/A
  //_dge/=_dsy
  //N/A

  return 0;
}

/*****************************************************************************/
