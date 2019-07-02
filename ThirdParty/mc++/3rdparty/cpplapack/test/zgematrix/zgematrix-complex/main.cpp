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
  
  CPPL::zgematrix A(M,N);
  for(int i=0; i<A.m; i++){ for(int j=0; j<A.n; j++){
    A(i,j) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
  }}
  
  cout << "A =\n" << A << endl;
  cout << "A*(1,0) =\n" << A*complex<double>(1.,0.) << endl;
  cout << "A*(0,1) =\n" << A*complex<double>(0.,1.) << endl;
  cout << "A/(1,0) =\n" << A/complex<double>(1.,0.) << endl;
  cout << "A/(0,1) =\n" << A/complex<double>(0.,1.) << endl;
  
  cout << "#### A*=(1,0); ####" << endl;
  A*=complex<double>(1.,0.);
  cout << "A =\n" << A << endl;
  cout << "#### A*=(0,1); ####" << endl;
  A*=complex<double>(0.,1.);
  cout << "A =\n" << A << endl;
  cout << "#### A/=(1,0); ####" << endl;
  A/=complex<double>(1.,0.);
  cout << "A =\n" << A << endl;
  cout << "#### A/=(0,1); ####" << endl;
  A/=complex<double>(0.,1.);
  cout << "A =\n" << A << endl;
  
  return 0;
}

/*****************************************************************************/
