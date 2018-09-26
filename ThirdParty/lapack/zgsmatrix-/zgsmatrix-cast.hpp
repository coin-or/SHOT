//=============================================================================
/*! convert to _zgematrix */
inline _zgematrix zgsmatrix::to_zgematrix() const
{VERBOSE_REPORT;
  zgematrix newmat( zgematrix(m,n).zero() );
  for(size_t c=0; c<data.size(); c++){
    const zcomponent& z =data[c];
    newmat(z.i,z.j) =z.v;
  }
  return _(newmat);
}
