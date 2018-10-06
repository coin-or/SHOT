//=============================================================================
/*! drovector*_dcovector operator */
inline double operator*(const drovector& rovec, const _dcovector& covec)
{VERBOSE_REPORT;
#ifdef  CPPL_DEBUG
  if(rovec.l!=covec.l){
    ERROR_REPORT;
    std::cerr << "These two vectors can not make a product." << std::endl
              << "Your input was (" << rovec.l << ") * (" << covec.l << ")." << std::endl;
    exit(1);
  }
#endif//CPPL_DEBUG
  
  double val( ddot_( rovec.l, rovec.array, 1, covec.array, 1 ) );
  
  covec.destroy();
  return val;
}
