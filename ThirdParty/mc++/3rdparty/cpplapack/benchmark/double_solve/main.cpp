/*****************************************************************************/
/*                               noname.cpp                                  */
/*****************************************************************************/
#undef CPPL_VERBOSE
#undef CPPL_DEBUG
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
  //int size(2500);
  int size(1000);
  CPPL::dgematrix A(size,size);
  CPPL::dcovector y(size);
  
  srand(time(NULL));
  for(int i=0; i<size; i++){
    for(int j=0; j<size; j++){
      A(i,j) =double(rand())/double(RAND_MAX);
    }
    y(i) =double(rand())/double(RAND_MAX);
  }
  
  clock_t t0, t1;
  
  t0=clock();
  A.dgesv(y);
  t1=clock();
  
  cout << "dgesv took "<< (1000./CLOCKS_PER_SEC)*(t1-t0) << "[ms]." << endl;
  
  return 0;
}

/*****************************************************************************/
