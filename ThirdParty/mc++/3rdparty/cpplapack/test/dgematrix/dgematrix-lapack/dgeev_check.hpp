//=============================================================================
void dgeev_check_value()
{
  std::cout << "############ check dgeev value ############" << std::endl;
  
  srand(time(NULL));
  int M(3);
  
  //// make dgematrix A  ////
  CPPL::dgematrix A(M,M);
  for(int i=0; i<A.m; i++){ for(int j=0; j<A.n; j++){
      A(i,j) =double( rand() /(RAND_MAX/10) );
    }}
  
  //// make wr wi vr ////
  std::vector<double> wr, wi;
  
  //// make A_original ////
  CPPL::dgematrix A_original(A);
  
  //// dgeev ////
  A.dgeev(wr, wi);
  
  //// print ////
  std::cout << "A_original=\n" << A_original << std::endl;
  for(int i=0; i<A.m; i++){
    std::cout << "#### " << i << "th eigen ####" << std::endl;
    std::cout << "wr=" << wr[i] <<std::endl;
    std::cout << "wi=" << wi[i] <<std::endl;
  }
}

//=============================================================================
void dgeev_check_right()
{
  std::cout << "############ check dgeev right ############" << std::endl;
  
  srand(time(NULL));
  int M(3);
  
  //// make dgematrix A  ////
  CPPL::dgematrix A(M,M);
  for(int i=0; i<A.m; i++){ for(int j=0; j<A.n; j++){
      A(i,j) =double( rand() /(RAND_MAX/10) );
    }}
  
  //// make wr wi vr ////
  std::vector<double> wr, wi;
  std::vector<CPPL::dcovector> vrr, vri;
  
  //// make A_original ////
  CPPL::dgematrix A_original(A);
  
  //// dgeev ////
  A.dgeev(wr, wi ,vrr, vri);
  
  //// print ////
  std::cout << "A_original=\n" << A_original << std::endl;
  for(int i=0; i<A.m; i++){
    std::cout << "#### " << i << "th eigen ####" << std::endl;
    std::cout << "wr=" << wr[i] <<std::endl;
    std::cout << "wi=" << wi[i] <<std::endl;
    std::cout << "vrr=\n" << vrr[i] <<std::endl;
    std::cout << "vri=\n" << vri[i] <<std::endl;
    std::cout << "Real[ [A]*{x} -lambda*{x} ] = (Should be zeros)\n"
              << A_original*vrr[i] -(wr[i]*vrr[i] - wi[i]*vri[i]) 
              << std::endl;
    std::cout << "Imag[ [A]*{x} -lambda*{x} ] = (Should be zeros)\n"
              << A_original*vri[i] -(wr[i]*vri[i] + wi[i]*vrr[i]) 
              << std::endl;
  }
}

//=============================================================================
void dgeev_check_left()
{
  std::cout << "############ check dgeev left ############" << std::endl;
  
  srand(time(NULL));
  int M(3);
  
  //// make dgematrix A  ////
  CPPL::dgematrix A(M,M);
  for(int i=0; i<A.m; i++){ for(int j=0; j<A.n; j++){
      A(i,j) =double( rand() /(RAND_MAX/10) );
    }}
  
  //// make wr wi vl ////
  std::vector<double> wr, wi;
  std::vector<CPPL::drovector> vlr, vli;
  
  //// make A_original ////
  CPPL::dgematrix A_original(A);
  
  //// dgeev ////
  A.dgeev(wr, wi ,vlr, vli);
  
  //// print ////
  std::cout << "A_original=\n" << A_original << std::endl;
  for(int i=0; i<A.m; i++){
    std::cout << "#### " << i << "th eigen ####" << std::endl;
    std::cout << "wr = " << wr[i] << std::endl;
    std::cout << "wi = " << wi[i] << std::endl;
    std::cout << "vlr = " << vlr[i];
    std::cout << "vli = " << vli[i] << std::endl;
    std::cout << "Real[ {x}*[A] -{x}*lambda ] = (Should be zeros)\n"
              << vlr[i]*A_original -(vlr[i]*wr[i] - vli[i]*wi[i]) 
              << std::endl;
    std::cout << "Imag[ {x}*[A] -{x}*lambda ] = (Should be zeros)\n"
              << vli[i]*A_original -(vli[i]*wr[i] + vlr[i]*wi[i]) 
              << std::endl;
  }
}
