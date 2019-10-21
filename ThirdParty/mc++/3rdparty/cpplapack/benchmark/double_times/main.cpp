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
  int size(500);
  CPPL::dgematrix A(size,size), B(size,size), C;
  
  srand(time(NULL));
  for(int i=0; i<size; i++){ for(int j=0; j<size; j++){
    A(i,j) =double(rand())/double(RAND_MAX);
    B(i,j) =double(rand())/double(RAND_MAX);
  }}
  
  clock_t t0, t1, t2;
  
  t0=clock();
  
  C=A*B;
  
  t1=clock();
  
  C.resize(size,size);
  for(int i=0; i<size; i++){
    for(int j=0; j<size; j++){
      C(i,j) =0.0;
      for(int k=0; k<size; k++){ C(i,j) +=A(i,k)*B(k,j); }
    }
  }
  
  t2=clock();
  
  cout << "\"A*B\"  took "<< (1000./CLOCKS_PER_SEC)*(t1-t0) << "[ms]." << endl;
  cout << "\"loop\" took "<< (1000./CLOCKS_PER_SEC)*(t2-t1) << "[ms]." << endl;
  
  return 0;
}

/*****************************************************************************/
