#include <Magick++.h>
#include <cpplapack.h>

//=============================================================================
int main(int argc, char** argv)
{
  //////// argc check ////////
  if(argc!=2){
    std::cerr << "[ERROR] invalid usage" << std::endl;
    std::cerr << "USAGE: " << argv[0] << " xxxx.dgematrix" << std::endl;
    exit(1);
  }
  
  //////// open ////////
  CPPL::dgematrix mat(argv[1]);
  std::cerr << "mat size = " << mat.m << "x" << mat.n << std::endl;
  Magick::Image img(Magick::Geometry(mat.n,mat.m),Magick::ColorGray(0));
  std::cerr << "img size = " << img.columns() << "x" << img.rows() << std::endl;
  
  //////// assign pixelcolor ////////
  int n(16);
  double maximum(fabs(damax(mat)));
  std::cerr << "maximum = " << maximum << std::endl;
  for(int i=0; i<mat.m; i++){
    for(int j=0; j<mat.n; j++){
      double x;
      x = ( log10(maximum)-log10(fabs(mat(i,j))) )/double(n);
      x = std::min(x,1.);
      img.pixelColor(j,i, Magick::ColorGray(x));
    }
  }
  
  //////// write ////////
  img.write(std::string(argv[1])+".png");
  return 0;
}
