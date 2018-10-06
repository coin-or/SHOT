//=============================================================================
/*! dgesvd_check */
void dgesvd_check()
{
  std::cout << "############ check dgesvd ############" << std::endl;
  
  srand(time(NULL));
  int M(4), N(3);
  
  //// make dgematrix A  ////
  CPPL::dgematrix A(M,N);
  for(int i=0; i<A.m; i++){ for(int j=0; j<A.n; j++){
	A(i,j) =double( rand() /(RAND_MAX/10) );
  }}
  
  //// make A_original ////
  CPPL::dgematrix A_original(A);
  
  //// make S, U and VT ////
  CPPL::dcovector S;
  CPPL::dgematrix U, VT;
  
  //// SVD A ////
  A.dgesvd(S,U,VT);
  
  //// print A, S, U, and VT ////
  std::cout << "A=\n" << A << std::endl;
  std::cout << "S=\n" << S << std::endl;
  std::cout << "U=\n" << U << std::endl;
  std::cout << "VT=\n" << VT << std::endl;
  
  CPPL::dgematrix S_matrix(M,N);
  S_matrix.zero();
  for(int i=0; i<std::min(M,N); i++){ S_matrix(i,i) =S(i); }
  std::cout << "S_matrix=\n" << S_matrix << std::endl;
  std::cout << "A_original=\n" << A_original << std::endl;
  std::cout << "U*S_matrix*VT=\n" << U*S_matrix*VT << std::endl;
}
