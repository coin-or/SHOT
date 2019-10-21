//=============================================================================
void dsygv_check_value()
{
  cout << "############ check dsygv value ############" << endl;
  
  srand(time(NULL));
  int N(3);
  
  //// make dsymatrix A ////
  CPPL::dsymatrix A(N);
  for(int i=0; i<A.n; i++){ for(int j=0; j<=i; j++){
    A(i,j) =double( rand() /(RAND_MAX/10) );
  }}
  
  //// make definite dsymatrix B ////
  CPPL::dsymatrix B(N);
  CPPL::dgematrix pd, ut(N,N);
  ut.zero();
  for(int i=0; i<ut.n; i++){ for(int j=0; j<=i; j++){
    ut(i,j) =double( rand() /(RAND_MAX/10) );
  }}
  pd=t(ut)*ut;
  for(int i=0; i<B.n; i++){ for(int j=0; j<=i; j++){
    B(i,j) =pd(i,j);
  }}
  
  //// make w ////
  vector<double> w;
  
  //// make A_original and B_original ////
  CPPL::dsymatrix A_original(A), B_original(B);
  
  //// dsygv ////
  A.dsygv(B, w);
  
  //// print ////
  cout << "A_original=\n" << A_original << endl;
  cout << "B_original=\n" << B_original << endl;
  for(int i=0; i<A.n; i++){
    cout << "#### " << i << "th eigen ####" << endl;
    cout << "w=" << w[i] <<endl;
  }
}

//=============================================================================
void dsygv_check_both()
{
  cout << "############ check dsygv value ############" << endl;
  
  srand(time(NULL));
  int N(3);
  
  //// make dsymatrix A ////
  CPPL::dsymatrix A(N);
  for(int i=0; i<A.n; i++){ for(int j=0; j<=i; j++){
    A(i,j) =double( rand() /(RAND_MAX/10) );
  }}
  
  //// make definite dsymatrix B ////
  CPPL::dsymatrix B(N);
  CPPL::dgematrix pd, ut(N,N);
  ut.zero();
  for(int i=0; i<ut.n; i++){ for(int j=0; j<=i; j++){
    ut(i,j) =double( rand() /(RAND_MAX/10) );
  }}
  pd=t(ut)*ut;
  for(int i=0; i<B.n; i++){ for(int j=0; j<=i; j++){
    B(i,j) =pd(i,j);
  }}
  
  //// make w ////
  vector<double> w;
  vector<CPPL::dcovector> v;
  
  //// make A_original and B_original ////
  CPPL::dsymatrix A_original(A), B_original(B);
  
  //// dsygv ////
  A.dsygv(B, w, v);
  
  //// print ////
  cout << "A_original=\n" << A_original << endl;
  cout << "B_original=\n" << B_original << endl;
  for(int i=0; i<A.n; i++){
    cout << "#### " << i << "th eigen ####" << endl;
    cout << "w=" << w[i] <<endl;
    cout << "v=\n" << v[i] <<endl;
    cout << "(A_original-w*B_original)*v=\n" << (A_original-w[i]*B_original)*v[i] << "<- Should be zeros" << endl;
  }
}
