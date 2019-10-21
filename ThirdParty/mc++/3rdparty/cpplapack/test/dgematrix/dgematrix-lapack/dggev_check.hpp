//=============================================================================
void dggev_check_value()
{
  std::cout << "############ check dggev value ############" << std::endl;
  
  srand(time(NULL));
  int M(3);
  
  //// make dgematrix A and B ////
  CPPL::dgematrix A(M,M), B(M,M);
  for(int i=0; i<A.m; i++){ for(int j=0; j<A.n; j++){
    A(i,j) =double( rand() /(RAND_MAX/10) );
    B(i,j) =double( rand() /(RAND_MAX/10) );
  }}
  
  //// make wr wi vr ////
  std::vector<double> wr, wi;
  
  //// make A_original and B_original ////
  CPPL::dgematrix A_original(A);
  CPPL::dgematrix B_original(B);
  
  //// dggev ////
  A.dggev(B, wr, wi);
  
  //// print ////
  std::cout << "A_original=\n" << A_original << std::endl;
  std::cout << "B_original=\n" << B_original << std::endl;
  for(int i=0; i<A.m; i++){
    std::cout << "#### " << i << "th eigen ####" << std::endl;
    std::cout << "wr=" << wr[i] <<std::endl;
    std::cout << "wi=" << wi[i] <<std::endl;
  }
}

//=============================================================================
void dggev_check_right()
{
  std::cout << "############ check dggev right ############" << std::endl;
  
  srand(time(NULL));
  int M(3);
  
  //// make dgematrix A and B ////
  CPPL::dgematrix A(M,M), B(M,M);
  for(int i=0; i<A.m; i++){ for(int j=0; j<A.n; j++){
    A(i,j) =double( rand() /(RAND_MAX/10) );
    B(i,j) =double( rand() /(RAND_MAX/10) );
  }}
  
  //// make wr wi vr ////
  std::vector<double> wr, wi;
  std::vector<CPPL::dcovector> vrr, vri;
  
  //// make A_original and B_original ////
  CPPL::dgematrix A_original(A);
  CPPL::dgematrix B_original(B);
  
  //// dggev ////
  A.dggev(B, wr, wi ,vrr, vri);
  
  //// print ////
  std::cout << "A_original=\n" << A_original << std::endl;
  std::cout << "B_original=\n" << B_original << std::endl;
  for(int i=0; i<A.m; i++){
    std::cout << "#### " << i << "th eigen ####" << std::endl;
    std::cout << "wr=" << wr[i] <<std::endl;
    std::cout << "wi=" << wi[i] <<std::endl;
    std::cout << "vrr=\n" << vrr[i] <<std::endl;
    std::cout << "vri=\n" << vri[i] <<std::endl;
    std::cout << "Real[ [A]*{x} -lambda*[B]*{x} ] = (Should be zeros)\n"
         << A_original*vrr[i]
      -(wr[i]*B_original*vrr[i] - wi[i]*B_original*vri[i])
         << std::endl;
    std::cout << "Imag[ [A]*{x} -lambda*[B]*{x} ] = (Should be zeros)\n"
         << A_original*vri[i]
      -(wr[i]*B_original*vri[i] + wi[i]*B_original*vrr[i])
         << std::endl;
  }
}

//=============================================================================
void dggev_check_left()
{
  std::cout << "############ check dggev left ############" << std::endl;
  
  srand(time(NULL));
  int M(3);
  
  //// make dgematrix A and B ////
  CPPL::dgematrix A(M,M), B(M,M);
  for(int i=0; i<A.m; i++){ for(int j=0; j<A.n; j++){
    A(i,j) =double( rand() /(RAND_MAX/10) );
    B(i,j) =double( rand() /(RAND_MAX/10) );
  }}
  
  //// make wr wi vl ////
  std::vector<double> wr, wi;
  std::vector<CPPL::drovector> vlr, vli;
  
  //// make A_original and B_original ////
  CPPL::dgematrix A_original(A);
  CPPL::dgematrix B_original(B);
  
  //// dggev ////
  A.dggev(B, wr, wi ,vlr, vli);
  
  //// print ////
  std::cout << "A_original=\n" << A_original << std::endl;
  std::cout << "B_original=\n" << B_original << std::endl;
  for(int i=0; i<A.m; i++){
    std::cout << "#### " << i << "th eigen ####" << std::endl;
    std::cout << "wr = " << wr[i] << std::endl;
    std::cout << "wi = " << wi[i] << std::endl;
    std::cout << "vlr = " << vlr[i];
    std::cout << "vli = " << vli[i] << std::endl;
    std::cout << "Real[ {x}*[A] -lambda*{x}*[B] ] = (Should be zeros)\n"
         << vlr[i]*A_original
      -(wr[i]*vlr[i]*B_original - wi[i]*vli[i]*B_original)
         << std::endl;
    std::cout << "Imag[ [A]*{x} -lambda*{x}*[B] ] = (Should be zeros)\n"
         << vli[i]*A_original
      -(wr[i]*vli[i]*B_original + wi[i]*vlr[i]*B_original)
         << std::endl;
  }
}
