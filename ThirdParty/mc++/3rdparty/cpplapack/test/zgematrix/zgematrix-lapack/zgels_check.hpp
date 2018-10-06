//=============================================================================
/*! zgels_check */
void zgels_check_vector()
{
  cout << "############ check zgels vector ############" << endl;
  
  srand(time(NULL));
  int M(3), N(2);
  
  //// make zgematrix A  ////
  CPPL::zgematrix A(M,N);
  for(int i=0; i<A.m; i++){ for(int j=0; j<A.n; j++){
    A(i,j) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
  }}
  
  //// make zcovector y  ////
  CPPL::zcovector y(M);
  for(int i=0; i<y.l; i++){
    y(i) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
  }
  
  //// make originals ////
  CPPL::zgematrix A_original(A);
  CPPL::zcovector y_original(y);
  
  //// zgels ////
  A.zgels(y);
  
  //// print ////
  cout << "######## zgels(vec) ########" << endl;
  cout << "A_original=\n" << A_original << endl;
  cout << "y_original=\n" << y_original << endl;
  cout << "A=\n" << A << endl;
  cout << "y=\n" << y << endl;
  cout << "A_original*y=\n" << A_original*y << endl;
  
  //// reset ////
  A =A_original;
  y =y_original;
  double r; //residual square
  
  //// zgels ////
  A.zgels(y,r);
  
  //// print ////
  cout << "######## zgels(vec, residual) ########" << endl;
  cout << "A_original=\n" << A_original << endl;
  cout << "y_original=\n" << y_original << endl;
  cout << "A=\n" << A << endl;
  cout << "y=\n" << y << endl;
  cout << "residual=" << r << endl;
  cout << "A_original*y=\n" << A_original*y << endl;
}

//==============================================================================
void zgels_check_matrix()
{
  cout << "############ check zgels matrix ############" << endl;
  
  srand(time(NULL));
  int M(3), N(2);
  
  //// make zgematrix A  ////
  CPPL::zgematrix A(M,N);
  for(int i=0; i<A.m; i++){ for(int j=0; j<A.n; j++){
    A(i,j) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
  }}

  //// make zgematrix Y  ////
  CPPL::zgematrix Y(M,M);
  for(int i=0; i<Y.m; i++){ for(int j=0; j<Y.n; j++){
    Y(i,j) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
  }}
   
  //// make originals ////
  CPPL::zgematrix A_original(A);
  CPPL::zgematrix Y_original(Y);
  
  //// zgels ////
  A.zgels(Y);
  
  //// print ////
  cout << "######## zgels(mat) ########" << endl;
  cout << "A_original=\n" << A_original << endl;
  cout << "Y_original=\n" << Y_original << endl;
  cout << "A=\n" << A << endl;
  cout << "Y=\n" << Y << endl;
  cout << "A_original*Y=\n" << A_original*Y << endl;
  
  //// reset ////
  A =A_original;
  Y =Y_original;
  CPPL::drovector R; //residual square
  
  //// zgels ////
  A.zgels(Y,R);
  
  //// print ////
  cout << "######## zgels(mat, residual) ########" << endl;
  cout << "A_original=\n" << A_original << endl;
  cout << "Y_original=\n" << Y_original << endl;
  cout << "A=\n" << A << endl;
  cout << "Y=\n" << Y << endl;
  cout << "residual=" << R << endl;
  cout << "A_original*Y=\n" << A_original*Y << endl;  
}
