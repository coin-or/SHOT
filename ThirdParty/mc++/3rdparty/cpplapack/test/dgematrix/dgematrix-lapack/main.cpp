//=============================================================================
#include <iostream>
#include <cstdlib>
#include <ctime>
#include "cpplapack.h"

#include "dgesv_check.hpp"
#include "dgels_check.hpp"
#include "dgelss_check.hpp"
#include "dgeev_check.hpp"
#include "dggev_check.hpp"
#include "dgesvd_check.hpp"
#include "dgglse_check.hpp"

//=============================================================================
/*! main */
int main(int argc, char** argv)
{
  //////// dgesv ////////
  dgesv_check_vector();
  dgesv_check_matrix();
  
  //////// dgels ////////
  dgels_check_vector();
  dgels_check_matrix();
  
  //////// dgelss ////////
  dgelss_check_vector();
  dgelss_check_matrix();
  
  //////// dgeev ////////
  dgeev_check_value();
  dgeev_check_right();
  dgeev_check_left();
  
  //////// dggev ////////
  dggev_check_value();
  dggev_check_right();
  dggev_check_left();
  
  //////// dgesvd ////////
  dgesvd_check();

  //////// dgglse ////////
  dgglse_check();
  
  return 0;
}
