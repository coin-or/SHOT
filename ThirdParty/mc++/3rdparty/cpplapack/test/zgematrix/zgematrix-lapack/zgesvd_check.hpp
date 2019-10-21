//=============================================================================
/*! zgesvd_check */
void zgesvd_check()
{
  cout << "############ check zgesvd ############" << endl;
  
  srand(time(NULL));
  int M(4), N(3);
  
  //// make zgematrix A  ////
  CPPL::zgematrix A(M,N);
  for(int i=0; i<A.m; i++){ for(int j=0; j<A.n; j++){
    A(i,j) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
  }}
  
  //// make A_original ////
  CPPL::zgematrix A_original(A);
  
  //// make S, U and VT ////
  CPPL::dcovector S;
  CPPL::zgematrix U, VT;
  
  //// SVD A ////
  A.zgesvd(S,U,VT);
  
  //// print A, S, U, and VT ////
  cout << "A=\n" << A << endl;
  cout << "S=\n" << S << endl;
  cout << "U=\n" << U << endl;
  cout << "VT=\n" << VT << endl;
  
  CPPL::zgematrix S_matrix(M,N);
  S_matrix.zero();
  for(int i=0; i<std::min(M,N); i++){ S_matrix(i,i) =S(i); }
  cout << "S_matrix=\n" << S_matrix << endl;
  cout << "A_original=\n" << A_original << endl;
  cout << "U*S_matrix*VT=\n" << U*S_matrix*VT << endl;
}
