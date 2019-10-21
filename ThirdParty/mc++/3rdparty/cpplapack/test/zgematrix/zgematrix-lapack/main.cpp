//====================================================================[include]
#include <iostream>
#include <cstdlib>
#include <ctime>
#include "cpplapack.h"
using namespace std;


#include "zgesv_check.hpp"
#include "zgels_check.hpp"
#include "zgelss_check.hpp"
#include "zgeev_check.hpp"
//#include "zggev_check.hpp"
#include "zgesvd_check.hpp"

//=======================================================================[main]
/*! main */
int main(int argc, char** argv)
{
  zgesv_check_vector();
  zgesv_check_matrix();
  
  zgels_check_vector();
  zgels_check_matrix();
  
  zgelss_check();
  
  zgeev_check_value();
  zgeev_check_right();
  zgeev_check_left();
  
  //zggev_check_value();
  //zggev_check_right();
  //zggev_check_left();
  
  zgesvd_check();
  
  return 0;
}
