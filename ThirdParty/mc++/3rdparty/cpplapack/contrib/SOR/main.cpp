/*****************************************************************************/
/*                                main.cpp                                   */
/*****************************************************************************/
//=============================================================================
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <vector>
#include "cpplapack.h"

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! solve */
int32_t sor
(
 const CPPL::dgsmatrix& A,
 const CPPL::dcovector& b,
 CPPL::dcovector& x,
 const double& eps
 )
{
  ///////////////////////////////////////////////
  ///////////////// mid values //////////////////
  ///////////////////////////////////////////////
  CPPL::dcovector r(b-A*x);
  CPPL::dcovector y(x.l);
  const double omega(1.);//Gauss-Siedel
  
  //////// norm ////////
  double norm_r, norm_r_min(DBL_MAX);
  const double norm_r_ini(fabs(damax(r)));
  std::cerr << "[NOTE]@sor: norm_r_ini=" << norm_r_ini << ", eps=" << eps<< std::endl;
  if( norm_r_ini<DBL_MIN ){
    std::cerr << "[NOTE]@sor: already converged. v(^^)" << std::endl;
    return 0;
  }
  
  ///////////////////////////////////////////////
  //////////////////// loop /////////////////////
  ///////////////////////////////////////////////
  int itc(1);
  int itmax(int(5.1*x.l));
  //int itmax(int(2.1*x.l));
  //int itmax(int(1.1*x.l));
  //int itmax(int(0.6*x.l));
  do{
    std::cerr << "** itc=" << itc << " ********************************************" << std::endl;
    for(long i=0; i<x.l; i++){
      //std::cerr << "++++ i=" << i << " ++++" << std::endl;
      double sigma =0.;
      for(std::vector< std::pair<long,long> >::const_iterator p=A.rows[i].begin(); p!=A.rows[i].end(); p++){
        if(p->first<i){
          sigma +=A.data[p->second].v*y(p->first);
        }
        else if(p->first>i){
          sigma +=A.data[p->second].v*x(p->first);
        }
      }
      double z =(b(i)-sigma)/A(i,i);
      y(i) =x(i) +omega*(z-x(i));
    }
    //////// update ////////
    x =y;
    //std::cerr << "x = " << t(x) << std::endl;
    
    //////// residual ////////
    r =b-A*x;
    //std::cerr << "r = " << t(r) << std::endl;
    
    //////// convergence check ////////
    norm_r =fabs(damax(r));
    std::cerr << "norm_r = " << norm_r << std::endl;
    if( isnan(norm_r) ){ break; }//failed
    if( !std::isnormal(norm_r) ){ break; }//failed
    if( !std::isfinite(norm_r) ){ break; }//failed
    if( norm_r>1e3*norm_r_ini ){ break; }//failed (getting so worse)
    if( norm_r<=eps ){//r satistied
      std::cerr << "[NOTE]@sor: converged. v(^^)  itc=" << itc << "/" << itmax << ", norm=" << norm_r << std::endl;
      return 0;
    }
  }while(++itc<itmax);
  
  //////// failed ////////
  std::cerr << "[NOTE]@sor: itc=" << itc << ", norm=" << norm_r << ", r_satisfied=" << (norm_r<=eps) << std::endl;
  std::cerr << "[NOTE]@sor: failed to converge. orz" << std::endl;
  return 1;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
int main(int argc, char** argv)
{
  ///////////////////////////////////////////////
  //////////////// set precision ////////////////
  ///////////////////////////////////////////////
  std::cout.precision(9);
  std::cout.setf(std::ios::scientific, std::ios::floatfield);
  std::cout.setf(std::ios::showpos);
  std::cerr.precision(3);
  std::cerr.setf(std::ios::scientific, std::ios::floatfield);
  std::cerr.setf(std::ios::showpos);
  
  ///////////////////////////////////////////////
  //////////////////// A,b //////////////////////
  ///////////////////////////////////////////////
  CPPL::dgsmatrix A;
  CPPL::dcovector b;
  bool file =false;
  ///////////////////////////
  ////////// read ///////////
  ///////////////////////////
  if(file){
    //A.read("A10.dge");  b.read("b10.dco");
  }
  
  ///////////////////////////
  ///////// random //////////
  ///////////////////////////
  else{//file==false
    std::cerr << "# making random matrix" << std::endl;
    const int size(1000);
    A.resize(size,size);
    b.resize(size);
    srand(time(NULL));
    for(int i=0; i<size; i++){
      for(int j=0; j<size; j++){
        if( rand()%5==0 ){
          A(i,j) =(double(rand())/double(RAND_MAX))*2.0 -1.0;
        }
      }
      A(i,i) +=1e-2*double(size);//generalize
      b(i) =(double(rand())/double(RAND_MAX))*1. -0.5;
    }
    A.write("A.dge");
    b.write("b.dco");
  }

  ///////////////////////////////////////////////
  ////////////////// direct /////////////////////
  ///////////////////////////////////////////////
  std::cerr << "# making solution with dgesv" << std::endl;
  CPPL::dgematrix A2(A.to_dgematrix());
  CPPL::dcovector b2(b);
  A2.dgesv(b2);
  b2.write("ans.dco");
  
  ///////////////////////////////////////////////
  ///////////////// iterative ///////////////////
  ///////////////////////////////////////////////
  //////// initial x ////////
  CPPL::dcovector x(b.l);
  //x.read("x_ini8.dco");
  x.zero();
  //////// eps ////////
  double eps(fabs(damax(b))*1e-6);
  std::cerr << "eps=" << eps << std::endl;
  //////// solve ////////
  if( sor(A, b, x, eps) ){
    std::cerr << "failed." << std::endl;
    x.write("x.dco");
    exit(1);
  }
  x.write("x.dco");
  std::cerr << "fabs(damax(err))=" << fabs(damax(b2-x)) << std::endl;
  
  return 0;
}
