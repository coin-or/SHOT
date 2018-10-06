#include <iostream>
#include "cpplapack.h"


int main(int argc, char *argv[])
{
  CPPL::zhematrix Z(4), W(4);
  // Assign values to lower triangle only.
  for( int i = 0; i < Z.n; i++ ) {
    for( int j = 0; j <=i; j++ ) {
      std::complex<double> x = std::complex<double>(i,j);
      if( i == j ) {
        Z(i,j) = std::complex<double>(i,0);
      } else {
        Z(i,j) = x;
      }
      std::cout << "(" << i << "," << j << "):Z(i,j)=" << Z(i,j) << std::endl;
    }
  }
  
  std::cout << "Z = " << Z << std::endl;

  // Assing values to upper triangle only.
  for( int i = 0; i < W.n; i++ ) {
    for( int j = i; j < W.n; j++ ) {
      std::complex<double> x = std::complex<double>(j,i);
      if( i == j ) { 
        W(i,j) = std::complex<double>(j,0);
      } else {
        W(i,j) = x;
      }
      std::cout << "(" << i << "," << j << "):W(i,j)=" << W(i,j) << std::endl;
    }
  }

  std::cout << "W = " << W << std::endl;

  std::cout << "Z+W = " << Z+W << "<-Should have all imaginary parts zero." << std::endl;
  std::cout << "Z-W = " << Z-W << "<-Should have all real parts zero." << std::endl;

  CPPL::zgematrix G(Z.n, Z.n);
  for( int i = 0; i < Z.n; i++ ) {
    for( int j = 0; j < Z.n; j++ ) {
      std::cout << "sqrt( " << Z(i,j) << " ) = " << std::sqrt( Z(i,j) ) << std::endl;
        G(i,j) = std::sqrt( Z(i,j) ) * std::sqrt( Z(i,j) );
    }
  }

  std::cout << "G-Z = " << G-Z << "<-Should be zero." << std::endl;

  W.write( "zhe.txt" );
  CPPL::zhematrix WW;
  WW.read( "zhe.txt" );
  std::cout << "W-WW = " << W-WW << "<-Should be zero." << std::endl;
  return 0;
}

