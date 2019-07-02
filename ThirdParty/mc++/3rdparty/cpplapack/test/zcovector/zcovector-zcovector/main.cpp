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
  
  CPPL::zcovector x(M), y(M), z;
  for(int i=0; i<x.l; i++){
    x(i) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
  }
  for(int i=0; i<y.l; i++){
    y(i) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
  }
  
  cout << "x =\n" << x << endl;
  cout << "y =\n" << y << endl;
  
  cout << "x+x =\n" << x+x << endl;
  cout << "x-x =\n" << x-x << endl;
  cout << "x%y =\n" << x%y << endl;
  
  cout << "#### z=x; ####" << endl;
  z=x;
  cout << "z =\n" << z << endl;
  cout << "#### z=x+x-x; ####" << endl;
  z=x+x-x;
  cout << "z =\n" << z << endl;
  cout << "#### z+=x; ####" << endl;
  z+=x;
  cout << "z =\n" << z << endl;
  cout << "#### z-=x; ####" << endl;
  z-=x;
  cout << "z =\n" << z << endl;

  return 0;
}

/*****************************************************************************/
