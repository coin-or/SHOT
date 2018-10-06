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
  
  CPPL::drovector a(N), b(N);
  CPPL::dsymatrix X(N), Y(N);
  for(int i=0; i<X.n; i++){ for(int j=0; j<X.n; j++){
	X(i,j) =double( rand() /(RAND_MAX/10) );
	Y(i,j) =double( rand() /(RAND_MAX/10) );
  }}
  for(int i=0; i<a.l; i++){
	a(i) =double( rand() /(RAND_MAX/10) );
	b(i) =double( rand() /(RAND_MAX/10) );
  }
  CPPL::dsymatrix Z(X+Y);
  
  //dro+dsy
    //N/A
  //dro-dsy
    //N/A
  //dro*dsy, _dsy+_dsy
  cout << "a*Z-a*X-a*Y =\n" << a*Z-a*X-a*Y << "<-Should be zero." << endl;

  //dro/dsy
    //N/A
  //dro=dsy
    //N/A
  //dro+=dsy
  	//N/A
  //dro-=dsy
  	//N/A
  //dro*=dsy
  	//N/A
  //dro/=dsy
  	//N/A
  
  //dro+_dsy
  	//N/A
  //dro-_dsy
  	//N/A
  //dro*_dsy, dro*dsy, dsy+dsy, _dsy-_dsy
  cout << "a*Z-a*(X+Y) =\n" << a*Z-a*(X+Y) << "<-Should be zero." << endl;
  //dro/_dsy
  	//N/A
  //dro=_dsy
  	//N/A
  //dro+=_dsy
  	//N/A
  //dro-=_dsy
  	//N/A
  //dro*=_dsy
  	//N/A
  //dro/=_dsy
  	//N/A
  
  //_dro+dsy
  	//N/A
  //_dro-dsy
  	//N/A
  //_dro*dsy, dro+dro, dro*dsy, _dsy-_dsy
  cout << "(a+b)*X-a*X-b*X =\n" << (a+b)*X-a*X-b*X << "<-Should be zero." << endl;
  //_dro/dsy
  	//N/A
  //_dro=dsy
  	//N/A
  //_dro+=dsy
  	//N/A
  //_dro-=dsy
  	//N/A
  //_dro*=dsy
  	//N/A
  //_dro/=dsy
  	//N/A

  //_dro+_dsy
  	//N/A
  //_dro-_dsy
  	//N/A
  //_dro*_dsy, dro-dro, dsy-dsy, dro*dsy, _dsy-_dsy, _dsy+_dsy
  cout << "(a-b)*(X-Y)-a*X+a*Y+b*X-b*Y =\n" << (a-b)*(X-Y)-a*X+a*Y+b*X-b*Y << "<-Should be zero." << endl;
  //_dro/_dsy
  	//N/A
  //_dro=_dsy
  	//N/A
  //_dro+=_dsy
  	//N/A
  //_dro-=_dsy
  	//N/A
  //_dro*=_dsy
  	//N/A
  //_dro/=_dsy
  	//N/A
  
  return 0;
}

/*****************************************************************************/
