//=============================================================================
/*! dsysv_check */
void dsysv_check_vector()
{
  cout << "############ check dsysv vector ############" << endl;
  
  srand(time(NULL));
  int N(3);
  
  //// make dsymatrix A  ////
  CPPL::dsymatrix A(N);
  for(int i=0; i<A.n; i++){ for(int j=0; j<A.n; j++){
    A(i,j) =double( rand() /(RAND_MAX/10) );
  }}
  
  //// make dcovector y ////
  CPPL::dcovector y(N);
  for(int i=0; i<y.l; i++){
    y(i) =double( rand() /(RAND_MAX/10) );
  }
  
  //// make A_original and y_original ////
  CPPL::dsymatrix A_original(A);
  CPPL::dcovector y_original(y);
  cout << "A_original=\n" << A_original << endl;
  cout << "y_original=\n" << y_original << endl;
  
  //// solve Ax=y ////
  A.dsysv(y);
    
  //// print A, y and A_original*y ////
  cout << "A=\n" << A << endl;
  cout << "y=\n" << y << endl;
  cout << "A_original*y=\n" << A_original*y << endl;
}

//=============================================================================
void dsysv_check_matrix()
{
  cout << "############ check dsysv matrix ############" << endl;
  
  srand(time(NULL));
  int N(3);
  

  //// make dsymatrix A  ////
  CPPL::dsymatrix A(N);
  for(int i=0; i<A.n; i++){ for(int j=0; j<A.n; j++){
    A(i,j) =double( rand() /(RAND_MAX/10) );
  }}
   
  //// make dsymatrix Y  ////
  CPPL::dgematrix Y(N,N);
  for(int i=0; i<Y.m; i++){ for(int j=0; j<Y.n; j++){
    Y(i,j) =double( rand() /(RAND_MAX/10) );
  }}
   
  //// make A_original and Y_original ////
  CPPL::dsymatrix A_original(A);
  CPPL::dgematrix Y_original(Y);
  cout << "A_original=\n" << A_original << endl;
  cout << "Y_original=\n" << Y_original << endl;
  
  //// solve AY=B ////
  A.dsysv(Y);
    
  //// print A, Y and A_original*Y ////
  cout << "A=\n" << A << endl;
  cout << "Y=\n" << Y << endl;
  cout << "A_original*Y=\n" << A_original*Y << endl;
}
