//=============================================================================
/*! dgglse_check */
void dgglse_check()
{
  std::cout << "############ check dgglse ############" << std::endl;
  
  srand(time(NULL));
  int M(3), N(4), P(2);
  
  //////// make A ////////
  CPPL::dgematrix A(M,N);
  for(int i=0; i<A.m; i++){
    for(int j=0; j<A.n; j++){
      A(i,j) =double( rand() /(RAND_MAX/10) );
    }
  }
  CPPL::dgematrix A0(A);
  
  //////// make B ////////
  CPPL::dgematrix B(P,N);
  for(int i=0; i<B.m; i++){
    for(int j=0; j<B.n; j++){
      B(i,j) =double( rand() /(RAND_MAX/10) );
    }
  }
  CPPL::dgematrix B0(B);
  
  //////// make c ////////
  CPPL::dcovector c(M);
  for(int i=0; i<c.l; i++){
    c(i) =double( rand() /(RAND_MAX/10) );
  }
  CPPL::dcovector c0(c);
  
  //////// make d ////////
  CPPL::dcovector d(P);
  for(int i=0; i<d.l; i++){
    d(i) =double( rand() /(RAND_MAX/10) );
  }
  CPPL::dcovector d0(d);
  
  //////// solve LSE ////////
  CPPL::dcovector x;
  A.dgglse(B,c,d,x);
  
  //// print ////
  std::cout << "t(c0-A0*x)=" << t(c0-A0*x) << std::endl;
  std::cout << "t(d0-B0*x)=" << t(d0-B0*x) << std::endl;
}
