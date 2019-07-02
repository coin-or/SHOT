//=============================================================================
/*! zgelss_check */
void zgelss_check()
{
  cout << "############ check zgelss ############" << endl;
  
  srand(time(NULL));
  int M(3), N(4);
  long RANK(0);
  double RCOND(-1.0);
  
  //// make zgematrix A  ////
  CPPL::zgematrix A(M,N);
  for(int i=0; i<A.m; i++){ for(int j=0; j<A.n; j++){
    A(i,j) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
  }}

  //// make dcovector b  ////
  CPPL::zcovector b(M);
  for(int i=0; i<b.l; i++){
    b(i) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
  }

  //// make dcovector s  ////
  CPPL::dcovector s;
   
  //// make A_original ////
  CPPL::zgematrix A_original(A);
  
  //// make b_original ////
  CPPL::zcovector b_original(b);
  
  //// zgels ////
  A.zgelss(b,s,RANK,RCOND);
  
  //// print ////
  cout << "A_original=\n" << A_original << endl;
  cout << "b_original=\n" << b_original << endl;
  cout << "A=\n" << A << endl;
  cout << "b=\n" << b << endl;
  cout << "s=\n" << s << endl;
  cout << "A_original*b=\n" << A_original*b << endl;
  cout << "RANK =" << RANK << endl;
}

