//=============================================================================
/*! dgels_check */
void dgels_check_vector()
{
  std::cout << "############ check dgels vector ############" << std::endl;
  
  srand(time(NULL));
  int M(3), N(2);
  
  //// make dgematrix A  ////
  CPPL::dgematrix A(M,N);
  for(int i=0; i<A.m; i++){ for(int j=0; j<A.n; j++){
    A(i,j) =double( rand() /(RAND_MAX/10) );
  }}
  
  //// make dcovector y  ////
  CPPL::dcovector y(M);
  for(int i=0; i<y.l; i++){
    y(i) =double( rand() /(RAND_MAX/10) );
  }
  
  //// make originals ////
  CPPL::dgematrix A_original(A);
  CPPL::dcovector y_original(y);
  
  //// dgels ////
  A.dgels(y);
  
  //// print ////
  std::cout << "######## dgels(vec) ########" << std::endl;
  std::cout << "A_original=\n" << A_original << std::endl;
  std::cout << "y_original=\n" << y_original << std::endl;
  std::cout << "A=\n" << A << std::endl;
  std::cout << "y=\n" << y << std::endl;
  std::cout << "A_original*y=\n" << A_original*y << std::endl;
  
  //// reset ////
  A =A_original;
  y =y_original;
  double r; //residual square
  
  //// dgels ////
  A.dgels(y,r);
  
  //// print ////
  std::cout << "######## dgels(vec, residual) ########" << std::endl;
  std::cout << "A_original=\n" << A_original << std::endl;
  std::cout << "y_original=\n" << y_original << std::endl;
  std::cout << "A=\n" << A << std::endl;
  std::cout << "y=\n" << y << std::endl;
  std::cout << "residual=" << r << std::endl;
  std::cout << "A_original*y=\n" << A_original*y << std::endl;
}

//==============================================================================
void dgels_check_matrix()
{
  std::cout << "############ check dgels matrix ############" << std::endl;
  
  srand(time(NULL));
  int M(3), N(2);
  
  //// make dgematrix A  ////
  CPPL::dgematrix A(M,N);
  for(int i=0; i<A.m; i++){ for(int j=0; j<A.n; j++){
    A(i,j) =double( rand() /(RAND_MAX/10) );
  }}

  //// make dgematrix Y  ////
  CPPL::dgematrix Y(M,M);
  for(int i=0; i<Y.m; i++){ for(int j=0; j<Y.n; j++){
    Y(i,j) =double( rand() /(RAND_MAX/10) );
  }}
   
  //// make originals ////
  CPPL::dgematrix A_original(A);
  CPPL::dgematrix Y_original(Y);
  
  //// dgels ////
  A.dgels(Y);
  
  //// print ////
  std::cout << "######## dgels(mat) ########" << std::endl;
  std::cout << "A_original=\n" << A_original << std::endl;
  std::cout << "Y_original=\n" << Y_original << std::endl;
  std::cout << "A=\n" << A << std::endl;
  std::cout << "Y=\n" << Y << std::endl;
  std::cout << "A_original*Y=\n" << A_original*Y << std::endl;
  
  //// reset ////
  A =A_original;
  Y =Y_original;
  CPPL::drovector R; //residual square
  
  //// dgels ////
  A.dgels(Y,R);
  
  //// print ////
  std::cout << "######## dgels(mat, residual) ########" << std::endl;
  std::cout << "A_original=\n" << A_original << std::endl;
  std::cout << "Y_original=\n" << Y_original << std::endl;
  std::cout << "A=\n" << A << std::endl;
  std::cout << "Y=\n" << Y << std::endl;
  std::cout << "residual=" << R << std::endl;
  std::cout << "A_original*Y=\n" << A_original*Y << std::endl;  
}
