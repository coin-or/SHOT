//=============================================================================
void dsyev_check_value()
{
  cout << "############ check dsyev value ############" << endl;
  
  srand(time(NULL));
  int N(3);
  
  //// make dsymatrix A  ////
  CPPL::dsymatrix A(N);
  for(int i=0; i<N; i++){ for(int j=0; j<N; j++){
    A(i,j) =double( rand() /(RAND_MAX/10) );
  }}
  
  //// make wr wi vr ////
  vector<double> w;
  
  //// make A_original ////
  CPPL::dsymatrix A_original(A);
  
  //// dsyev ////
  A.dsyev(w);
  //A.dsyev(w,1);
  
  //// print ////
  cout << "A=\n" << A << endl;
  cout << "A_original=\n" << A_original << endl;
  for(int i=0; i<N; i++){
    cout << "#### " << i << "th eigen ####" << endl;
    cout << "w=" << w[i] <<endl;
  }
}

//=============================================================================
void dsyev_check_right()
{
  cout << "############ check dsyev right ############" << endl;
  
  srand(time(NULL));
  int N(3);
  
  //// make dsymatrix A  ////
  CPPL::dsymatrix A(N);
  for(int i=0; i<A.n; i++){ for(int j=0; j<A.n; j++){
    A(i,j) =double( rand() /(RAND_MAX/10) );
  }}
  
  //// make wr wi vr ////
  vector<double> w;
  vector<CPPL::dcovector> v;
  
  //// make A_original ////
  CPPL::dsymatrix A_original(A);
  
  //// dsyev ////
  A.dsyev(w,v);
  
  //// print ////
  cout << "A_original=\n" << A_original << endl;
  for(int i=0; i<A.n; i++){
    cout << "#### " << i << "th eigen ####" << endl;
    cout << "w=" << w[i] <<endl;
    cout << "v=\n" << v[i] <<endl;
    cout << "[A]*{x} -lambda*{x} = (Should be zeros)\n"
         << A_original*v[i] -w[i]*v[i] << endl;
  }
}

//=============================================================================
void dsyev_check_left()
{
  cout << "############ check dsyev left ############" << endl;
  
  srand(time(NULL));
  int N(3);
  
  //// make dsymatrix A  ////
  CPPL::dsymatrix A(N);
  for(int i=0; i<A.n; i++){ for(int j=0; j<A.n; j++){
    A(i,j) =double( rand() /(RAND_MAX/10) );
  }}
  
  //// make wr wi vl ////
  vector<double> w;
  vector<CPPL::drovector> v;
  
  //// make A_original ////
  CPPL::dsymatrix A_original(A);
  
  //// dsyev ////
  A.dsyev(w, v);
  
  //// print ////
  cout << "A_original=\n" << A_original << endl;
  for(int i=0; i<A.n; i++){
    cout << "#### " << i << "th eigen ####" << endl;
    cout << "w = " << w[i] << endl;
    cout << "v = " << v[i] << endl;
    cout << "{x}*[A] -{x}*lambda = (Should be zeros)\n"
         << v[i]*A_original -v[i]*w[i] << endl;
  }
}
