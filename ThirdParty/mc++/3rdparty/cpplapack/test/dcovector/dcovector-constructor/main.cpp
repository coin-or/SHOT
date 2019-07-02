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
  
  CPPL::dcovector x;
  cout << "x || l=" << x.l << ", array=" << x.array << endl;
  
  
  CPPL::dcovector y(M);
  for(int i=0; i<y.l; i++){
    y(i) =double( rand() /(RAND_MAX/10) );
  }
  cout << "y || l=" << y.l << ", array=" << y.array << endl;
  cout << y << endl;
  
  CPPL::dcovector z(y);
  cout << "z || l=" << z.l << ", array=" << z.array << endl;
  cout << z << endl;
  
  return 0;
}

/*****************************************************************************/
