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
  int M(4);
  
  CPPL::dcovector x(M);
  CPPL::drovector y(M);
  
  for(int i=0; i<x.l; i++){
	x(i) =double( rand() /(RAND_MAX/10) );
  }
  for(int i=0; i<y.l; i++){
	y(i) =double( rand() /(RAND_MAX/10) );
  }
  
  cout << "x =\n" << x << endl;
  cout << "y =\n" << y << endl;
  cout << "x*y =\n" << x*y << endl;
  
  return 0;
}

/*****************************************************************************/
