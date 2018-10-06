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
  
  CPPL::zcovector x(M);
  for(int i=0; i<x.l; i++){
    x(i) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
  }
  
  cout << "x =\n" << x << endl;
  cout << "t(x) =\n" << CPPL::t(x) << endl;
  cout << "conj(x) =\n" << CPPL::conj(x) << endl;
  cout << "conjt(x) =\n" << CPPL::conjt(x) << endl;
  cout << "nrm2(x) =\n" << CPPL::nrm2(x) << endl;
  cout << "idamax(x)=\n" << CPPL::idamax(x) << endl;
  cout << "damax(x)=\n" << CPPL::damax(x) << endl;
  
  return 0;
}

/*****************************************************************************/
