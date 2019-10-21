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
  CPPL::dgematrix A(size,size), B;
  
  srand(time(NULL));
  for(int i=0; i<size; i++){ for(int j=0; j<size; j++){
      A(i,j) =double(rand())/double(RAND_MAX);
  }}
  
  clock_t t0, t1, t2, t3;
  
  t0=clock();//////////////////////////////////////////////////////////////////
  
  B=A;
  
  t1=clock();//////////////////////////////////////////////////////////////////
  
  B.resize(size,size);
  for(int i=0; i<size*size; i++){
    B.array[i] =A.array[i];
  }
  
  t2=clock();//////////////////////////////////////////////////////////////////
  
  B.resize(size,size);
  memcpy(B.array, A.array, size*size*sizeof(double));
  
  t3=clock();//////////////////////////////////////////////////////////////////
  
  std::cout << "\"B=A (dcopy)\"   took "<< (1000./CLOCKS_PER_SEC)*(t1-t0) << "[ms]." << std::endl;
  std::cout << "\"loop\"          took "<< (1000./CLOCKS_PER_SEC)*(t2-t1) << "[ms]." << std::endl;
  std::cout << "\"memcpy\"        took "<< (1000./CLOCKS_PER_SEC)*(t3-t2) << "[ms]." << std::endl;
  
  return 0;
}

/*****************************************************************************/
