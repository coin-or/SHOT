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
  
  CPPL::dsymatrix A(N);
  CPPL::dgematrix X(N,N), Y(N,N), Z(N,N);
  for(int i=0; i<A.n; i++){ for(int j=0; j<=i; j++){
    A(i,j) =double( rand() /(RAND_MAX/10) );
  }}
  for(int i=0; i<A.n; i++){ for(int j=0; j<A.n; j++){
    Z(i,j) =double( rand() /(RAND_MAX/10) );
  }}
  for(int i=0; i<A.n; i++){ for(int j=0; j<A.n; j++){
    X(i,j) = -A(i,j);
    Y(i,j) = A(i,j);
  }}
  
  cout << "A =\n" << A << endl;
  cout << "X =\n" << X << endl;
  cout << "Y =\n" << Y << endl;
  cout << "Z =\n" << Z << endl;

  
  //dsy+dge
  cout << "A+X =\n" << A+X << "<-Should be zero." << endl;
  //dsy-dge
  cout << "A-Y =\n" << A-Y << "<-Should be zero." << endl;
  //dsy*dge, t(_dge), t(dge), _dge*dsy, _dge-_dge
  cout << "A*Y =\n" << t(A*Y)-t(Y)*A << "<-Should be zero." << endl;

  //dsy+_dge, -dge
  cout << "A+(-Y) =\n" << A+(-Y) << "<-Should be zero." << endl;
  //dsy-_dge, -dge
  cout << "A-(-X) =\n" << A-(-X) << "<-Should be zero." << endl;
  //dsy*_dge, dge+dge, dsy*dge, _dge+_dge, _dge-_dge
  cout << "A*(X+Z) - (A*X+A*Z) =\n" << A*(X+Z) - (A*X+A*Z) << "<-Should be zero." << endl;
  
  //_dsy+dge, -dsy
  cout << "(-A)+Y =\n" << (-A)+Y << "<-Should be zero." << endl;
  //_dsy-dge, -dsy
  cout << "(-A)-X =\n" << (-A)-X << "<-Should be zero." << endl;
  //_dsy*dge, -dsy, dsy*dge, _dge+_dge
  cout << "(-A)*Z+(A*Z) =\n" << ((-A)*Z+(A*Z)) << "<-Should be zero." << endl;
  
  //_dsy+_dge, -dsy, -dge
  cout << "(-A)+(-X) =\n" << (-A)+(-X) << "<-Should be zero." << endl;
  //_dsy-_dge, -dsy, -dge
  cout << "(-A)-(-Y) =\n" << (-A)-(-Y) << "<-Should be zero." << endl;
  //_dsy*_dge, -dsy, -dge, dsy*dge, _dge*_dge
  cout << "(-A)*(-Z)-(A*Z) =\n" << (-A)*(-Z)-(A*Z) << "<-Should be zero." << endl;
  
  return 0;
}

/*****************************************************************************/
