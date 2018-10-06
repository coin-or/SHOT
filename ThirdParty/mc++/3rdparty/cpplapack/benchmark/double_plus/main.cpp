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

//=======================================================================[main]
/*! main */
int main(int argc, char** argv)
{
  int size(3000);
  CPPL::dgematrix A(size,size), B(size,size), C;
  
  srand(time(NULL));
  for(int i=0; i<size; i++){ for(int j=0; j<size; j++){
    A(i,j) =double(rand())/double(RAND_MAX);
    B(i,j) =double(rand())/double(RAND_MAX);
  }}
  
  clock_t t0, t1, t2, t3;
  
  t0=clock();
  
  C=A+B;
  
  t1=clock();
  
  C.resize(size,size);
  for(int i=0; i<size*size; i++){
    C.array[i] =A.array[i]+B.array[i];
  }
  
  t2=clock();
  
  C=B;
  daxpy_(size*size, 1., A.array, 1, C.array, 1);
  
  t3=clock();
  
  std::cout << "\"A+B\"   took "<< (1000./CLOCKS_PER_SEC)*(t1-t0) << "[ms]." << std::endl;
  std::cout << "\"loop\"  took "<< (1000./CLOCKS_PER_SEC)*(t2-t1) << "[ms]." << std::endl;
  std::cout << "\"daxpy\" took "<< (1000./CLOCKS_PER_SEC)*(t3-t2) << "[ms]." << std::endl;
  
  return 0;
}

/*****************************************************************************/
