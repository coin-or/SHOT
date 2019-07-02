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
  int M(5), N(5), KL(2), KU(3);

  CPPL::dsymatrix X(N), Y(N), Z(N);
  CPPL::dgbmatrix A(M,N,KL,KU);
  A.zero();
  X.zero();
  Y.zero();
  Z.zero();
  for(int i=0; i<A.m; i++){
    for(int j=std::max(0l,i-A.kl); j<std::min(A.n,i+A.ku+1); j++){
      if( i>=j && (std::max(0l,j-A.kl) <= i && i < std::min(A.n,j+A.ku+1) )){
        A(i,j) = double( rand() /(RAND_MAX/10) );
        A(j,i) = A(i,j);
        X(i,j) = -A(i,j);
        Y(i,j) = A(i,j);
      }
    }
  }
  for(int i=0; i<Z.n; i++){
    for(int j=0; j<Z.n; j++){
      Z(i,j) = double( rand() /(RAND_MAX/10) );
    }
  }
  
  cout << "A =\n" << A << endl;
  cout << "X =\n" << X << endl;
  cout << "Y =\n" << Y << endl;
  cout << "Z =\n" << Z << endl;
  
  
  //dgb+dsy
  cout << "A+X =\n" << A+X << "<-Should be zero." << endl;
  //dgb-dsy
  cout << "A-Y =\n" << A-Y << "<-Should be zero." << endl;
  //dgb*dsy
  cout << "t(A*Z)-Z*t(A) =\n" << t(A*Z)-Z*t(A) << "<-Should be zero." << endl;

  //dgb/dsy
  //N/A
  //dgb=dsy
  //N/A
  //dgb+=dsy
  //N/A
  //dgb-=dsy
  //N/A
  //dgb*=dsy
  //N/A
  //dgb/=dsy
  //N/A
  
  //dgb+_dsy, -dsy
  cout << "A+(-Y) =\n" << A+(-Y) << "<-Should be zero." << endl;
  //dgb-_dsy, -dsy
  cout << "A-(-X) =\n" << A-(-X) << "<-Should be zero." << endl;
  //dgb*_dsy, dsy+dsy, dgb*dsy, _dge+_dge, _dge-_dge
  cout << "A*(X+Z) - (A*X+A*Z) =\n" << A*(X+Z) - (A*X+A*Z) << "<-Should be zero." << endl;
  //dgb/_dsy
  //N/A
  //dgb=_dsy
  //N/A
  //dgb+=_dsy
  //N/A
  //dgb-=_dsy
  //N/A
  //dgb*=_dsy
  //N/A
  //dgb/=_dsy
  //N/A
  
  //_dgb+dsy, -dgb
  cout << "(-A)+Y =\n" << (-A)+Y << "<-Should be zero." << endl;
  //_dgb-dsy, -dgb
  cout << endl << endl << endl << endl;
  cout << "(-A)-X =\n" << (-A)-X << "<-Should be zero." << endl;
  cout << endl << endl << endl << endl;
  //_dgb*dsy, -dgb, dgb*dsy, _dge+_dge
  cout << "(-A)*Z+(A*Z) =\n" << ((-A)*Z+(A*Z)) << "<-Should be zero." << endl;
  //_dgb/dsy
  //N/A
  //_dgb=dsy
  //N/A
  //_dgb+=dsy
  //N/A
  //_dgb-=dsy
  //N/A
  //_dgb*=dsy
  //N/A
  //_dgb/=dsy
  //N/A
  
  //_dgb+_dsy, -dgb, -dsy
  cout << "(-A)+(-X) =\n" << (-A)+(-X) << "<-Should be zero." << endl;
  //_dgb-_dsy, -dgb, -dsy
  cout << "(-A)-(-Y) =\n" << (-A)-(-Y) << "<-Should be zero." << endl;
  //_dgb*_dsy, -dgb, -dsy, dgb*dsy, _dge-_dge
  cout << "(-A)*(-Z)-(A*Z) =\n" << (-A)*(-Z)-(A*Z) << "<-Should be zero." << endl;
  //_dgb/_dsy
  //N/A
  //_dgb=_dsy
  //N/A
  //_dgb+=_dsy
  //N/A
  //_dgb-=_dsy
  //N/A
  //_dgb*=_dsy
  //N/A
  //_dgb/=_dsy
  //N/A
  
  return 0;
}

/*****************************************************************************/
