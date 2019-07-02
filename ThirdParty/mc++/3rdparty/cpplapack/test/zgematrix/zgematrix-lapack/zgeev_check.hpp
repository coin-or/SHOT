//=============================================================================
void zgeev_check_value()
{
  cout << "############ check zgeev value ############" << endl;
  
  srand(time(NULL));
  int M(3);
  
  //// make zgematrix A  ////
  CPPL::zgematrix A(M,M);
  for(int i=0; i<A.m; i++){ for(int j=0; j<A.n; j++){
    A(i,j) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
  }}
  
  //// make w ////
  vector< complex<double> > w;
  
  //// make A_original ////
  CPPL::zgematrix A_original(A);
  
  //// zgeev ////
  A.zgeev(w);
  
  //// print ////
  cout << "A_original=\n" << A_original << endl;
  for(int i=0; i<A.m; i++){
    cout << "#### " << i << "th eigen ####" << endl;
    cout << "w=" << w[i] <<endl;
  }
}

//=============================================================================
void zgeev_check_right()
{
  cout << "############ check zgeev right ############" << endl;
  
  srand(time(NULL));
  int M(3);
  
  //// make zgematrix A  ////
  CPPL::zgematrix A(M,M);
  for(int i=0; i<A.m; i++){ for(int j=0; j<A.n; j++){
    A(i,j) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
  }}
  
  //// make w v ////
  vector< complex<double> > w;
  vector<CPPL::zcovector> vr;
  
  //// make A_original ////
  CPPL::zgematrix A_original(A);
  
  //// zgeev ////
  A.zgeev(w, vr);
  
  //// print ////
  cout << "A_original=\n" << A_original << endl;
  for(int i=0; i<A.m; i++){
    cout << "#### " << i << "th eigen ####" << endl;
    cout << "w=" << w[i] <<endl;
    cout << "vr=\n" << vr[i] <<endl;
    cout << "[A]*{x} -lambda*{x} = (Should be zeros)\n"
         << A_original*vr[i] -w[i]*vr[i] << endl;
  }
}

//=============================================================================
void zgeev_check_left()
{
  cout << "############ check zgeev left ############" << endl;
  
  //srand(time(NULL));
  int M(3);
  
  //// make zgematrix A  ////
  CPPL::zgematrix A(M,M);
  for(int i=0; i<A.m; i++){ for(int j=0; j<A.n; j++){
    A(i,j) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
  }}
  
  //// make w v ////
  vector< complex<double> > w;
  vector<CPPL::zrovector> vl;
  
  //// make A_original ////
  CPPL::zgematrix A_original(A);
  
  //// zgeev ////
  A.zgeev(w, vl);
  
  //// print ////
  cout << "A_original=\n" << A_original << endl;
  for(int i=0; i<A.m; i++){
    cout << "#### " << i << "th eigen ####" << endl;
    cout << "w= " << w[i] << endl;
    cout << "vl = " << vl[i] << endl;
    cout << "{x}*[A] -{x}*lambda = (Should be zeros)\n"
         << vl[i]*A_original -vl[i]*w[i] << endl;
  }
}
