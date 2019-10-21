//=============================================================================
/*! zhesv_check */
void zhesv_check_vector()
{
  cout << "############ check zhesv vector ############" << endl;
  
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
  
  //// make zcovector y ////
  CPPL::zcovector y(N);
  for(int i=0; i<y.l; i++){
    y(i) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
  }
  
  //// make A_original and y_original ////
  CPPL::zhematrix A_original(A);
  CPPL::zcovector y_original(y);
  cout << "A_original=\n" << A_original << endl;
  cout << "y_original=\n" << y_original << endl;
  
  //// solve Ax=y ////
  A.zhesv(y);
    
  //// print A, y and A_original*y ////
  cout << "A=\n" << A << endl;
  cout << "y=\n" << y << endl;
  cout << "A_original*y=\n" << A_original*y << endl;
}

//=============================================================================
void zhesv_check_matrix()
{
  cout << "############ check zhesv matrix ############" << endl;
  
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
  
  //// make zhematrix Y  ////
  CPPL::zgematrix Y(N,N);
  for(int i=0; i<Y.m; i++){ for(int j=0; j<Y.n; j++){
    Y(i,j) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
  }}
   
  //// make A_original and Y_original ////
  CPPL::zhematrix A_original(A);
  CPPL::zgematrix Y_original(Y);
  cout << "A_original=\n" << A_original << endl;
  cout << "Y_original=\n" << Y_original << endl;
  
  //// solve AY=B ////
  A.zhesv(Y);
    
  //// print A, Y and A_original*Y ////
  cout << "A=\n" << A << endl;
  cout << "Y=\n" << Y << endl;
  cout << "A_original*Y=\n" << A_original*Y << endl;
}
