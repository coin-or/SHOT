/*****************************************************************************/
/*                                main.cpp                                   */
/*****************************************************************************/
//=============================================================================
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <vector>
#include "cpplapack.h"

//=============================================================================
/*! apply J */
inline
void rotate
(
 double& x,
 double& y,
 const double& co,
 const double& si
 )
{
  double _x = co*x + si*y;
  y =-si*x + co*y;
  x=_x;
}

//=============================================================================
/*! construct J */
inline
void make_rotator
(
 const double& x,
 const double& y,
 double& co,
 double& si
 )
{
  if( fabs(y)<DBL_MIN ){
    co = 1.;
    si = 0.;
  }
  else if( fabs(y)>fabs(x) ){
    double t = x/y;
    si = 1./sqrt( 1.+t*t );
    co = si*t;
  }
  else{
    double t = y/x;
    co = 1./sqrt( 1.+t*t );
    si = co*t;
  }
}


///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
/*! solve */
int32_t gmres
(
 const CPPL::dgsmatrix& A,
 const CPPL::dcovector& b,
 CPPL::dcovector& x,
 const double& eps
 )
{
  ///////////////////////////////////////////////
  //////////////// preconditioner ///////////////
  ///////////////////////////////////////////////
  CPPL::dgbmatrix Minv(x.l, x.l, 0, 0);
  
  //////// no precondition ////////
  Minv.identity();
  
  ///////////////////////////////////////////////
  ///////////////// mid values //////////////////
  ///////////////////////////////////////////////
  long m(10);//restart number
  CPPL::dcovector r(b-A*x);
  CPPL::dcovector s(m+1), co(m+1), si(m+1), w;
  std::vector<CPPL::dcovector> v(m+1);
  CPPL::dgematrix H(m+1,m);
  //H.zero();
  //co.zero();
  //si.zero();
  //s.zero();
  
  //////// norm ////////
  double norm_r, norm_r_min(DBL_MAX);
  const double norm_r_ini(fabs(damax(r)));
  std::cerr << "[NOTE]@gmres: norm_r_ini=" << norm_r_ini << ", eps=" << eps<< std::endl;
  if( norm_r_ini<DBL_MIN ){
    std::cerr << "[NOTE]@gmres: already converged. v(^^)" << std::endl;
    return 0;
  }
  
  ///////////////////////////////////////////////
  //////////////////// loop /////////////////////
  ///////////////////////////////////////////////
  int itc(1);
  //int itmax(int(2.1*x.l));
  int itmax(int(1.1*x.l));
  //int itmax(int(0.6*x.l));
  do{
    std::cerr << "** itc=" << itc << " ********************************************" << std::endl;
    //////// 0 ////////
    v[0] =r/nrm2(r);
    s.zero();
    s(0) =nrm2(r);
    
    for(long i=0; i<m; i++){
      //std::cerr << "++++ i=" << i << " ++++" << std::endl;
      w =A*v[i];
      w =Minv*w;
      for(long k=0; k<i+1; k++){
        H(k,i) =w%v[k];
        w -=H(k,i)*v[k];
      }
      H(i+1,i) =nrm2(w);
      v[i+1] =w/H(i+1,i);
      
      //// J,s ////
      for(long k=0; k<i; k++){
        rotate(H(k,i), H(k+1,i), co(k), si(k));
      }
      make_rotator( H(i,i), H(i+1,i), co(i), si(i) );
      //std::cerr << "co = " << t(co) << std::endl; std::cerr << "si = " << t(si) << std::endl;
      rotate( H(i,i), H(i+1,i), co(i), si(i) );//necessary
      //std::cerr << "H =\n" << H << std::endl;
      rotate( s(i), s(i+1), co(i), si(i) );
      //std::cerr << "s = " << t(s) << std::endl;
    }
    //for(long i=0; i<m+1; i++){ for(long j=i+1; j<m+1; j++){ std::cerr << "vv = " << v[i]%v[j] << std::endl; } }// v check
    //std::cerr << "H =\n" << H << std::endl;
    //std::cerr << "s =" << t(s) << std::endl;
    //for(long i=0; i<m+1; i++){ std::cerr << "v["<<i<<"] =" << t(v[i]) << std::flush; }
    
    //////// y ////////
    CPPL::dcovector y(s);
    for(long i=m-1; i>=0; i--){
      y(i) /= H(i,i);
      for(long j=i-1; j>=0; j--){
        y(j) -= H(j,i) * y(i);
      }
    }
    //std::cerr << "H*y = " << t(H*y) << std::endl;    
    //std::cerr << "s   = " << t(s) << std::endl;
    //std::cerr << "y = " << t(s) << std::endl;
    
    //////// update ////////
    for(long i=0; i<m; i++){
      x += v[i] * y(i);
    }
    //std::cerr << "x = " << t(x) << std::endl;
    
    //////// residual ////////
    r =b-A*x;
    r =Minv*r;
    //std::cerr << "r = " << t(r) << std::endl;
    
    //////// convergence check ////////
    norm_r =fabs(damax(r));
    std::cerr << "norm_r = " << norm_r << std::endl;
    if( isnan(norm_r) ){ break; }//failed
    if( !std::isnormal(norm_r) ){ break; }//failed
    if( !std::isfinite(norm_r) ){ break; }//failed
    if( norm_r>1e3*norm_r_ini ){ break; }//failed (getting so worse)
    if( norm_r<=eps ){//r satistied
      std::cerr << "[NOTE]@gmres: converged. v(^^)  itc=" << itc << "/" << itmax << ", norm=" << norm_r << std::endl;
      return 0;
    }
  }while(++itc<itmax);
  
  //////// failed ////////
  std::cerr << "[NOTE]@gmres: itc=" << itc << ", norm=" << norm_r << ", r_satisfied=" << (norm_r<=eps) << std::endl;
  std::cerr << "[NOTE]@gmres: failed to converge. orz" << std::endl;
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
  //const bool file =true;
  const bool file =false;
  ///////////////////////////
  ////////// read ///////////
  ///////////////////////////
  if(file){
    A.read("A8.dge");  b.read("b8.dco");
    //A.read("A10.dge");  b.read("b10.dco");
    //A.read("A44.dge"); b.read("b44.dco");
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
      A(i,i) +=1e-2*double(size);//generalization
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
  if( gmres(A, b, x, eps) ){
    std::cerr << "failed." << std::endl;
    x.write("x.dco");
    exit(1);
  }
  x.write("x.dco");
  std::cerr << "fabs(damax(err))=" << fabs(damax(b2-x)) << std::endl;
  
  return 0;
}
