//=============================================================================
void zggev_check_value()
{
  cout << "############ check zggev value ############" << endl;
  
  srand(time(NULL));
  int M(3);
  
  //// make zgematrix A and B ////
  CPPL::zgematrix A(M,M), B(M,M);
  for(int i=0; i<A.m; i++){ for(int j=0; j<A.n; j++){
    A(i,j) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
    B(i,j) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
  }}
  
  //// make wr wi vr ////
  vector< complex<double> > w;
  
  //// make A_original and B_original ////
  CPPL::zgematrix A_original(A);
  CPPL::zgematrix B_original(B);
  
  //// zggev ////
  A.zggev(w);
  
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
void zggev_check_right()
{
  cout << "############ check zggev right ############" << endl;
  
  srand(time(NULL));
  int M(3);
  
  //// make zgematrix A and B ////
  CPPL::zgematrix A(M,M), B(M,M);
  for(int i=0; i<A.m; i++){ for(int j=0; j<A.n; j++){
    A(i,j) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
    B(i,j) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
  }}
  
  //// make wr wi vr ////
  vector<double> wr, wi;
  vector<CPPL::dcovector> vrr, vri;
  
  //// make A_original and B_original ////
  CPPL::zgematrix A_original(A);
  CPPL::zgematrix B_original(B);
  
  //// zggev ////
  A.zggev(B, wr, wi ,vrr, vri);
  
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
void zggev_check_left()
{
  cout << "############ check zggev left ############" << endl;
  
  srand(time(NULL));
  int M(3);
  
  //// make zgematrix A and B ////
  CPPL::zgematrix A(M,M), B(M,M);
  for(int i=0; i<A.m; i++){ for(int j=0; j<A.n; j++){
    A(i,j) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
    B(i,j) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
  }}
  
  //// make wr wi vl ////
  vector<double> wr, wi;
  vector<CPPL::drovector> vlr, vli;
  
  //// make A_original and B_original ////
  CPPL::zgematrix A_original(A);
  CPPL::zgematrix B_original(B);
  
  //// zggev ////
  A.zggev(B, wr, wi ,vlr, vli);
  
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
