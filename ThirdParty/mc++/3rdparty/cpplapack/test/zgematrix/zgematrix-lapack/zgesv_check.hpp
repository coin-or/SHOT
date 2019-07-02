//=============================================================================
/*! zgesv_check */
void zgesv_check_vector()
{
  cout << "############ check zgesv vector ############" << endl;
  
  srand(time(NULL));
  int M(3);
  
  //// make zgematrix A  ////
  CPPL::zgematrix A(M,M);
  for(int i=0; i<A.m; i++){ for(int j=0; j<A.n; j++){
    A(i,j) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
  }}
  
  //// make zcovector y ////
  CPPL::zcovector y(M);
  for(int i=0; i<y.l; i++){
    y(i) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
  }
  
  //// make A_original and y_original ////
  CPPL::zgematrix A_original(A);
  CPPL::zcovector y_original(y);
  cout << "A_original=\n" << A_original << endl;
  cout << "y_original=\n" << y_original << endl;
  
  //// solve Ax=y ////
  A.zgesv(y);
    
  //// print A, y and A_original*y ////
  cout << "A=\n" << A << endl;
  cout << "y=\n" << y << endl;
  cout << "A_original*y=\n" << A_original*y << endl;
}

//=============================================================================
void zgesv_check_matrix()
{
  cout << "############ check zgesv matrix ############" << endl;
  
  srand(time(NULL));
  int M(3);
  

  //// make zgematrix A  ////
  CPPL::zgematrix A(M,M);
  for(int i=0; i<A.m; i++){ for(int j=0; j<A.n; j++){
    A(i,j) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
  }}
   
  //// make zgematrix Y  ////
  CPPL::zgematrix Y(M,M);
  for(int i=0; i<Y.m; i++){ for(int j=0; j<Y.n; j++){
    Y(i,j) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
  }}
   
  //// make A_original and Y_original ////
  CPPL::zgematrix A_original(A);
  CPPL::zgematrix Y_original(Y);
  cout << "A_original=\n" << A_original << endl;
  cout << "Y_original=\n" << Y_original << endl;
  
  //// solve AY=B ////
  A.zgesv(Y);
    
  //// print A, Y and A_original*Y ////
  cout << "A=\n" << A << endl;
  cout << "Y=\n" << Y << endl;
  cout << "A_original*Y=\n" << A_original*Y << endl;
}
