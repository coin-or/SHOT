//=============================================================================
void dggev_check_value()
{
  cout << "############ check dggev value ############" << endl;
  
  srand(time(NULL));
  int M(3);
  
  //// make dgematrix A and B ////
  CPPL::dgematrix A(M,M), B(M,M);
  for(int i=0; i<A.m; i++){ for(int j=0; j<A.n; j++){
    A(i,j) =double( rand() /(RAND_MAX/10) );
    B(i,j) =double( rand() /(RAND_MAX/10) );
  }}
  
  //// make wr wi vr ////
  vector<double> wr, wi;
  
  //// make A_original and B_original ////
  CPPL::dgematrix A_original(A);
  CPPL::dgematrix B_original(B);
  
  //// dggev ////
  A.dggev(B, wr, wi);
  
  //// print ////
  cout << "A_original=\n" << A_original << endl;
  cout << "B_original=\n" << B_original << endl;
  for(int i=0; i<A.m; i++){
    cout << "#### " << i << "th eigen ####" << endl;
    cout << "wr=" << wr[i] <<endl;
    cout << "wi=" << wi[i] <<endl;
  }
}

//=============================================================================
void dggev_check_right()
{
  cout << "############ check dggev right ############" << endl;
  
  srand(time(NULL));
  int M(3);
  
  //// make dgematrix A and B ////
  CPPL::dgematrix A(M,M), B(M,M);
  for(int i=0; i<A.m; i++){ for(int j=0; j<A.n; j++){
    A(i,j) =double( rand() /(RAND_MAX/10) );
    B(i,j) =double( rand() /(RAND_MAX/10) );
  }}
  
  //// make wr wi vr ////
  vector<double> wr, wi;
  vector<CPPL::dcovector> vrr, vri;
  
  //// make A_original and B_original ////
  CPPL::dgematrix A_original(A);
  CPPL::dgematrix B_original(B);
  
  //// dggev ////
  A.dggev(B, wr, wi ,vrr, vri);
  
  //// print ////
  cout << "A_original=\n" << A_original << endl;
  cout << "B_original=\n" << B_original << endl;
  for(int i=0; i<A.m; i++){
    cout << "#### " << i << "th eigen ####" << endl;
    cout << "wr=" << wr[i] <<endl;
    cout << "wi=" << wi[i] <<endl;
    cout << "vrr=\n" << vrr[i] <<endl;
    cout << "vri=\n" << vri[i] <<endl;
    cout << "Real[ [A]*{x} -lambda*[B]*{x} ] = (Should be zeros)\n"
         << A_original*vrr[i]
      -(wr[i]*B_original*vrr[i] - wi[i]*B_original*vri[i])
         << endl;
    cout << "Imag[ [A]*{x} -lambda*[B]*{x} ] = (Should be zeros)\n"
         << A_original*vri[i]
      -(wr[i]*B_original*vri[i] + wi[i]*B_original*vrr[i])
         << endl;
  }
}

//=============================================================================
void dggev_check_left()
{
  cout << "############ check dggev left ############" << endl;
  
  srand(time(NULL));
  int M(3);
  
  //// make dgematrix A and B ////
  CPPL::dgematrix A(M,M), B(M,M);
  for(int i=0; i<A.m; i++){ for(int j=0; j<A.n; j++){
    A(i,j) =double( rand() /(RAND_MAX/10) );
    B(i,j) =double( rand() /(RAND_MAX/10) );
  }}
  
  //// make wr wi vl ////
  vector<double> wr, wi;
  vector<CPPL::drovector> vlr, vli;
  
  //// make A_original and B_original ////
  CPPL::dgematrix A_original(A);
  CPPL::dgematrix B_original(B);
  
  //// dggev ////
  A.dggev(B, wr, wi ,vlr, vli);
  
  //// print ////
  cout << "A_original=\n" << A_original << endl;
  cout << "B_original=\n" << B_original << endl;
  for(int i=0; i<A.m; i++){
    cout << "#### " << i << "th eigen ####" << endl;
    cout << "wr = " << wr[i] << endl;
    cout << "wi = " << wi[i] << endl;
    cout << "vlr = " << vlr[i];
    cout << "vli = " << vli[i] << endl;
    cout << "Real[ {x}*[A] -lambda*{x}*[B] ] = (Should be zeros)\n"
         << vlr[i]*A_original
      -(wr[i]*vlr[i]*B_original - wi[i]*vli[i]*B_original)
         << endl;
    cout << "Imag[ [A]*{x} -lambda*{x}*[B] ] = (Should be zeros)\n"
         << vli[i]*A_original
      -(wr[i]*vli[i]*B_original + wi[i]*vlr[i]*B_original)
         << endl;
  }
}
