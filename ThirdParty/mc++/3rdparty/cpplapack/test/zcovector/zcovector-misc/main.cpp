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
  int L(4);
  
  CPPL::zcovector x(L), y;
  for(int i=0; i<x.l; i++){
    x(i) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
  }
  
  cout << "x =\n" << x << endl;
  cout << "#### y.copy(x) ####" << endl;
  y.copy(x);
  cout << "y =\n" << y << endl;
  
  cout << "#### y.clear() ####" << endl;
  y.clear();
  cout << "y =\n" << y << endl;
  
  cout << "#### y.resize(2) & y.zero() ####" << endl;
  y.resize(2);
  y.zero();
  cout << "y =\n" << y << endl;
  
  return 0;
}

/*****************************************************************************/
