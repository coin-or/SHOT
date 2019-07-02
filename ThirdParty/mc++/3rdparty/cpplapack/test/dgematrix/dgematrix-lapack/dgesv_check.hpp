//=============================================================================
/*! dgesv_check */
void dgesv_check_vector()
{
  std::cout << "############ check dgesv vector ############" << std::endl;
  
  srand(time(NULL));
  int M(3);
  
  //// make dgematrix A  ////
  CPPL::dgematrix A(M,M);
  for(int i=0; i<A.m; i++){ for(int j=0; j<A.n; j++){
    A(i,j) =double( rand() /(RAND_MAX/10) );
  }}
  
  //// make dcovector y ////
  CPPL::dcovector y(M);
  for(int i=0; i<y.l; i++){
    y(i) =double( rand() /(RAND_MAX/10) );
  }
  
  //// make A_original and y_original ////
  CPPL::dgematrix A_original(A);
  CPPL::dcovector y_original(y);
  std::cout << "A_original=\n" << A_original << std::endl;
  std::cout << "y_original=\n" << y_original << std::endl;
  
  //// solve Ax=y ////
  A.dgesv(y);
    
  //// print A, y and A_original*y ////
  std::cout << "A=\n" << A << std::endl;
  std::cout << "y=\n" << y << std::endl;
  std::cout << "A_original*y=\n" << A_original*y << std::endl;
}

void dgesv_check_matrix()
{
  std::cout << "############ check dgesv matrix ############" << std::endl;
  
  srand(time(NULL));
  int M(3);
  

  //// make dgematrix A  ////
  CPPL::dgematrix A(M,M);
  for(int i=0; i<A.m; i++){ for(int j=0; j<A.n; j++){
    A(i,j) =double( rand() /(RAND_MAX/10) );
  }}
   
  //// make dgematrix Y  ////
  CPPL::dgematrix Y(M,M);
  for(int i=0; i<Y.m; i++){ for(int j=0; j<Y.n; j++){
    Y(i,j) =double( rand() /(RAND_MAX/10) );
  }}
   
  //// make A_original and Y_original ////
  CPPL::dgematrix A_original(A);
  CPPL::dgematrix Y_original(Y);
  std::cout << "A_original=\n" << A_original << std::endl;
  std::cout << "Y_original=\n" << Y_original << std::endl;
  
  //// solve AY=B ////
  A.dgesv(Y);
    
  //// print A, Y and A_original*Y ////
  std::cout << "A=\n" << A << std::endl;
  std::cout << "Y=\n" << Y << std::endl;
  std::cout << "A_original*Y=\n" << A_original*Y << std::endl;
}
