//=============================================================================
void zheev_check_value()
{
  cout << "############ check zheev value ############" << endl;
  
  srand(time(NULL));
  int N(3);
  
  //// make zhematrix A  ////
  CPPL::zhematrix A(N);
  for(int i=0; i<A.n; i++){
    for(int j=0; j<i; j++){
      A(i,j) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
    }
    A(i,i) =complex<double>(rand()/(RAND_MAX/10), 0.0);
  }
  
  //// make w ////
  vector<double> w;
  
  //// make A_original ////
  CPPL::zhematrix A_original(A);
  
  //// zheev ////
  A.zheev(w);
  //A.zheev(w,1);
  
  //// print ////
  cout << "A=\n" << A << endl;
  cout << "A_original=\n" << A_original << endl;
  for(int i=0; i<N; i++){
    cout << "#### " << i << "th eigen ####" << endl;
    cout << "w=" << w[i] <<endl;
  }
}

//=============================================================================
void zheev_check_right()
{
  cout << "############ check zheev right ############" << endl;
  
  srand(time(NULL));
  int N(3);
  
  //// make zhematrix A  ////
  CPPL::zhematrix A(N);
  for(int i=0; i<A.n; i++){
    for(int j=0; j<i; j++){
      A(i,j) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
    }
    A(i,i) =complex<double>(rand()/(RAND_MAX/10), 0.0);
  }
  
  //// make wr wi vr ////
  vector<double> w;
  vector<CPPL::zcovector> v;
  
  //// make A_original ////
  CPPL::zhematrix A_original(A);
  
  //// zheev ////
  A.zheev(w,v);
  
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
void zheev_check_left()
{
  cout << "############ check zheev left ############" << endl;
  
  srand(time(NULL));
  int N(3);
  
  //// make zhematrix A  ////
  CPPL::zhematrix A(N);
  for(int i=0; i<A.n; i++){
    for(int j=0; j<i; j++){
      A(i,j) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
    }
    A(i,i) =complex<double>(rand()/(RAND_MAX/10), 0.0);
  }
  
  //// make wr wi vl ////
  vector<double> w;
  vector<CPPL::zrovector> v;
  
  //// make A_original ////
  CPPL::zhematrix A_original(A);
  
  //// zheev ////
  A.zheev(w, v);
  
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
