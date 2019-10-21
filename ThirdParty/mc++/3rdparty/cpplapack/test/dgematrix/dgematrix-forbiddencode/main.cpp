//====================================================================[include]
#include <iostream>
#include <cstdlib>
#include <ctime>
#include "cpplapack.h"
using namespace std;

void random( CPPL::drovector & x ) {
  for(int i=0; i<x.l; i++){
        x(i) =double( rand() /(RAND_MAX/10) );
  }
}

void random( CPPL::dcovector & x ) {
  for(int i=0; i<x.l; i++){
        x(i) =double( rand() /(RAND_MAX/10) );
  }
}

void random( CPPL::dgematrix & A ) {
  for(int i=0; i<A.m; i++){ for(int j=0; j<A.n; j++){
    A(i,j) =double( rand() /(RAND_MAX/10) );
  }}
}

void random( CPPL::dsymatrix & A ) {
  for(int i=0; i<A.n; i++){ for(int j=0; j<A.n; j++){
    A(i,j) =double( rand() /(RAND_MAX/10) );
  }}
}

void random( CPPL::dgbmatrix & A ) {
  for(int i=0; i<A.m; i++){
    for(int j=std::max(0l,i-A.kl); j<std::min(A.n,i+A.ku+1); j++){
      A(i,j) =double( rand() /(RAND_MAX/10) );
    }
  }
}

void random( CPPL::dgsmatrix & A ) {
  for(int i=0; i<A.m; i++){
    for(int j=0; j<=i; j++){
      if(rand()%2==0){
        A(i,j) =double( rand() / (RAND_MAX/10) );
      }
    }
  }
}

//=======================================================================[main]
/*! main */

int main( int argc, char * argv[] )
{
  srand(time(NULL));
  
  int M(9), N(11);
  int KL(2), KU(1);
  int CAP(4);

  CPPL::dgematrix A(M,N);
  CPPL::dgbmatrix B(M,N,KL,KU);
  CPPL::dgsmatrix C(M,N,CAP);
  CPPL::dsymatrix D(M);
  CPPL::drovector x(M);
  CPPL::dcovector y(M);

  random( A );
  random( B );
  random( C );
  random( D );
  random( x );
  random( y );

  std::cout << "A =\n" << A << std::endl;
  std::cout << "C =\n" << C << std::endl;
  std::cout << "t(C) =\n" << t(C) << std::endl;
  
  /*
  //////// cause memory leak ////////
  t( A );
  t( B );
  t( C );
  t( D );
  t( x );
  t( y );

  A + B;
  + A;
  i( D );
  y + t( x );
  C * t(C);
  */
  return 0;
}

