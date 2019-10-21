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
  CPPL::dgematrix A(size,size);
  
  srand(time(NULL));
  for(int i=0; i<size; i++){ for(int j=0; j<size; j++){
      A(i,j) =double(rand())/double(RAND_MAX);
  }}
  
  clock_t t0, t1, t2, t3;
  /*
  t0=clock();
  
  A.zero();
  
  t1=clock();
  
  for(int i=0; i<size*size; i++){
    A.array[i] =0.;
  }
  */
  t2=clock();
  
  memset(A.array, 0, size*size*sizeof(double));
  
  t3=clock();
  A.write("aho");
  
  std::cout << "\"A.zero()\"   took "<< (1000./CLOCKS_PER_SEC)*(t1-t0) << "[ms]." << std::endl;
  std::cout << "\"loop\"  took "<< (1000./CLOCKS_PER_SEC)*(t2-t1) << "[ms]." << std::endl;
  std::cout << "\"memset\" took "<< (1000./CLOCKS_PER_SEC)*(t3-t2) << "[ms]." << std::endl;
  
  return 0;
}

/*****************************************************************************/
