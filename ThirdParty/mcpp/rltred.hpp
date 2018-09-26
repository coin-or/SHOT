// Copyright (C) 2009-2016 Benoit Chachuat, Imperial College London.
// All Rights Reserved.
// This code is published under the Eclipse Public License.

//TODO: 
//- [TO DO] Simultaneous RLT reduction w.r.t. candidate variables
//BUGS: 
//- [RESOLVED] For division reduction, constants need to be taken into account

#ifndef MC__RLTRED_HPP
#define MC__RLTRED_HPP

#include "ffunc.hpp"
#include "mclapack.hpp"

#undef MC__RLTRED_DEBUG

namespace mc
{
//! @brief C++ structure for comparing variables in a factorable program
////////////////////////////////////////////////////////////////////////
//! mc::lt_RLTVar is a C++ structure for ordering variables in an RLT
//! map.
////////////////////////////////////////////////////////////////////////
struct lt_RLTVar
////////////////////////////////////////////////////////////////////////
{
  typedef std::pair< const FFVar*, typename FFOp::TYPE > t_RLTVar;

  bool operator()
    ( const t_RLTVar& Var1,
      const t_RLTVar& Var2 ) const
    {
      // Order RLT variables w.r.t. their RLT operation types first
      if( Var1.second < Var2.second ) return true;
      if( Var1.second > Var2.second ) return false;
      // Order RLT variables w.r.t. to their index next
      return lt_FFVar()( Var1.first, Var2.first );
    }
};

//! @brief C++ base class for reduced RLT generation in a DAG
////////////////////////////////////////////////////////////////////////
//! mc::RLTRED is a C++ base class implementing the reduced RLT
//! approach by Liberti et al. to identifying redundant constraints
//! in a DAG based on linear equality constraints.
////////////////////////////////////////////////////////////////////////
class RLTRED
{
public:
  typedef typename FFVar::pt_idVar pt_idVar;
  typedef std::set< const FFVar*, lt_FFVar > t_Vars;
  typedef std::pair< const FFOp*, unsigned int > pt_Op;
  typedef std::list< const FFOp* > l_Ops;
  typedef std::set< const FFOp*,  lt_FFOp > t_Ops;
  typedef std::multimap< const FFVar*, const FFOp*, lt_FFVar > t_Edges;
  typedef std::pair< const FFVar*, typename FFOp::TYPE > t_RLTVar;
  typedef std::multimap< const t_RLTVar, const FFOp*, lt_RLTVar > t_RLTRed;
  
private:
  //! @brief Pointer to DAG
  FFGraph* _dag;

  //! @brief Subset of linear operations
  t_Ops _linTerms;
  //! @brief Subset of variables participating in linear operations
  t_Vars _linVars;
  //! @brief Map of linear operations and participating variables
  t_Edges _linEdges;
  //! @brief Subset of variables for RLT constraint search
  t_Vars _RLTVars;
  //! @brief Subset of edges for RLT constraint search
  t_Edges _RLTEdges;

  // subsets of bilinear terms
  t_Ops _bilTerms;
  // subsets of fractional terms
  t_Ops _divTerms;
  // subsets of square terms
  t_Ops _sqrTerms;
  // subsets of square root terms
  t_Ops _sqrtTerms;

  //! @brief Map of RLT constraints and corresponding multiplier variable
  t_RLTRed _RLTRed;
  //! @brief Coefficient matrix of "hidden" linear (reduction) expressions
  CPPL::dgematrix _coefRed;
  //! @brief New variables/operations participating in "hidden" linear (reduction) expressions
  std::map< const FFOp*, unsigned, lt_FFOp > _varRed;
  //! @brief Existing (linear combinations of) variables participating in "hidden" linear (reduction) expressions
  std::vector<FFVar*> _rhsRed;
  //! @brief DAG variables representing "hidden" linear (reduction) expressions in DAG
  std::vector<const FFVar*> _ctrRed;

  //! @brief Whether a given RLT variable has been assigned to a linear term and which term
  std::vector<pt_Op> _VarAssigned;
  //! @brief Whether a given RLT variable has been visited
  std::vector<unsigned> _VarVisited;
  //! @brief Whether a given linear term has been visited
  std::vector<unsigned> _TermVisited;

public:
  
  //! @brief Default Constructor
  RLTRED
    ( FFGraph* dag )
    : _dag( dag )
    {}

  //! @brief Destructor
  ~RLTRED()
    { for( auto itv=_varRed.begin(); itv!=_varRed.end(); ++itv ) delete itv->first;
      for( auto itv=_rhsRed.begin(); itv!=_rhsRed.end(); ++itv ) delete *itv; }

  //! @brief Return a map of reduction constraints for the expressions in lhs
  t_RLTRed& search
    ( const std::vector<const FFVar*>& vlhs, const bool add2dag=true );

  //! @brief Return a map of reduction constraints for the expressions in lhs
  t_RLTRed& search
    ( const unsigned nlhs, const FFVar* lhs, const bool add2dag=true );

  //! @brief Return a vector of pointers to the reduction "hidden" constraints found
  const std::vector<const FFVar*>& constraints
    ()
    const
    { return _ctrRed; }

  //! @brief Options of mc::RLTRED
  struct Options
  {
    //! @brief Constructor
    Options():
      LEVEL(PRIMSEQ), NODIV(false), SVDTOL(1e2*machprec()), DISPLAY(0)
      {}
    //! @brief Enumeration type for reduced RLT strategy
    enum RLTVARS{
      PRIMSEQ=0,//!< Reduced RLT with primary variables as operands only treated sequentially
      FULLSEQ,	//!< Reduced RLT with primary and auxiliary variables as operands treated sequentially
      PRIMSIM,	//!< Reduced RLT with primary variables as operands only treated simultaneously
      FULLSIM	//!< Reduced RLT with primary and auxiliary variables as operands treated simultaneously
    };
    //! @brief Reduced RLT variable selection
    RLTVARS LEVEL;
    //! @brief Whether or not to consider divider variables in addition to mutliplier variables
    bool NODIV;
    //! @brief Tolerance in SVD for hidden linear constraint detection
    double SVDTOL;
    //! @brief Display level
    unsigned DISPLAY;
  } options;

private:
  //! @brief Define subsets and map of linear operations and participating variables
  void _extract_linearity
    ( const l_Ops& Ops, const std::vector<const FFVar*>& vlhs );
  //! @brief Define subsets of bilinear, fractional, square and square root terms
  void _target_nonlinearity
    ( const l_Ops& Ops );
  //! @brief Search for redution constraints using a bi-partite graph for each candidate variable sequentially
  void _search_sequential
    ( const l_Ops& Ops );
  //! @brief Search for redution constraints using a bi-partite graph for all candidate variable simultaneously
  void _search_simultaneous
    ( const l_Ops& Ops );
  //! @brief Identify variables not yet participating in any nonlinear terms with variable <a>idMult</a>, and create corresponding bigraph edges
  void _bigraph_RLT
    ( const t_RLTVar& VarRed, t_Vars& RLTVars, t_Edges& RLTEdges )
    const;
  //! @brief Identify variables not yet participating in any nonlinear terms multiplied by variable <a>idMult</a>, and create corresponding bigraph edges
  void _bigraph_RLT_mult
    ( const FFVar* VarMult, t_Vars &linVars, t_Edges &linEdges )
    const;
  //! @brief Identify variables not yet participating in any nonlinear terms divided by variable <a>idDiv</a>, and create corresponding bigraph edges
  void _bigraph_RLT_div
    ( const FFVar* VarDiv, t_Vars &linVars, t_Edges &linEdges )
    const;
  //! @brief Identify valid reduction constraints with the candidate reduction variable <a>VarRed</a>
  void _reduction_RLT
    ( const t_RLTVar& VarRed );
  //! @brief Determine whether an augmented path can be found that emanates from the linear term <a>pOp<\a>
  bool _augpath_RLT
    ( const pt_Op& pOp );

  //! @brief Add reduced constraint expressions to DAG and display
  void _add_to_dag
    ( const std::vector<const FFVar*>& vlhs );
  //! @brief Process all terms in reduced constraint
  void _process_constraint
    ( const unsigned iRow, const std::vector<const FFVar*>& vlhs, const t_RLTVar& RLTVar,
      const FFOp*RLTOp );
  //! @brief Process one term in reduced constraint
  void _process_term
    ( const unsigned iRow, const FFOp::TYPE& RLTOp, const FFVar& RLTVar,
      const FFVar& linVar, const double coef );

  //! @brief Create subset of operations of given type
  t_Ops _subset_op
    ( const l_Ops& Ops, const unsigned int nOp, const typename FFOp::TYPE*typeOp )
    const;
  //! @brief Create subset of operations of given type
  t_Ops _subset_op
    ( const l_Ops& Ops, const typename FFOp::TYPE&typeOp )
    const;
  //! @brief Create subset of variables participating in operations Ops and sunmap with their defining operations 
  void _subset_var_edge
    ( const std::vector<const FFVar*>& vlhs, const t_Ops& Ops,
      t_Vars& Vars, t_Edges& Edges )
    const;
  //! @brief Test if a variable is corresponds to a dependent
  bool _is_rhs
    ( const std::vector<const FFVar*>& vlhs, const FFVar& varRes )
    const;
};
/*
inline std::pair< const unsigned, const FFVar* >
RLTRED::add
( const unsigned nlhs, const FFVar* lhs )
{
  std::vector<const FFVar*> vlhs;
  for( unsigned i=0; i<nlhs; i++ ) vlhs.push_back( lhs+i );
  auto vlhsred = add( vlhs );
  const unsigned nlhsred = vlhsred.size()
  const FFVar* lhsred = new const FFVar[ nlhsred ];
  for( unsigned i=0; i<nlhsred; i++ ) lhsred[i] = *vlhsred[i];
  return std::make_pair( nlhsred, lhsred );
}

inline std::vector<const FFVar*>
RLTRED::add
( const std::vector<const FFVar*>& vlhs )
{
}
*/

inline typename RLTRED::t_RLTRed&
RLTRED::search
( const unsigned nlhs, const FFVar* lhs, const bool add2dag )
{
  std::vector<const FFVar*> vlhs;
  for( unsigned i=0; i<nlhs; i++ ) vlhs.push_back( lhs+i );
  return search( vlhs, add2dag );
}

inline typename RLTRED::t_RLTRed&
RLTRED::search
( const std::vector<const FFVar*>& vlhs, const bool add2dag )
{
  // Generate list of operation for constraint LHS and particpating variables
  l_Ops Ops = _dag->subgraph( vlhs );

  // Define subsets and map of linear operations and participating variables
  _extract_linearity( Ops, vlhs );

  // Define subsets of bilinear, fractional, square and square root terms
  _target_nonlinearity( Ops );

  // Bi-partite graph approach to identifying reduction constraints
  switch( options.LEVEL ){
   case Options::PRIMSEQ: case Options::FULLSEQ: _search_sequential( Ops );   break;
   case Options::PRIMSIM: case Options::FULLSIM: _search_simultaneous( Ops ); break;
  }

  // Add reduced constraints to DAG
  if( add2dag ) _add_to_dag( vlhs );

  return _RLTRed;
}

inline void
RLTRED::_add_to_dag
( const std::vector<const FFVar*>& vlhs )
{
  // RLT did not give any reduction constraints
  _ctrRed.clear();
  if( _RLTRed.begin() == _RLTRed.end() ){
    if( options.DISPLAY >= 1 )
      std::cout << "\nNO RLT REDUCTION CONSTRAINTS FOUND\n";
    return;
  }

  // Display "raw" reduction constraints
  if( options.DISPLAY >= 2 ){
    std::cout << "\nRLT REDUCTION CONSTRAINTS:\n";
    for( auto it = _RLTRed.begin(); it != _RLTRed.end(); ++it ){
      std::cout << "  " << *(*it).first.first << "  <";
      switch( (*it).first.second ){
       case FFOp::TIMES:
        std::cout << "MULT"; break;
       case FFOp::DIV:
        std::cout << "DIV";  break;
       default:
        throw std::runtime_error( "Internal error" );
      }
      std::cout << ">  " << *(*it).second->pres << " := " << *(*it).second << std::endl;
    }
  }

  // Coefficient matrix and right-hand-side vector holding the "hidden" linear relationships
  const unsigned nRLTRed = _RLTRed.size();
  _coefRed.resize( nRLTRed, nRLTRed ); _coefRed.zero(); // Oversize and initialize to zero
  for( auto itv=_rhsRed.begin(); itv!=_rhsRed.end(); ++itv ) delete *itv;
  _rhsRed.assign( nRLTRed, 0 );
  for( auto itv=_varRed.begin(); itv!=_varRed.end(); ++itv ) delete itv->first;
  _varRed.clear(); 

  // Fill-in coefficient matrix and right-hand-side vector
  auto itRed = _RLTRed.begin();
  for( unsigned iRow = 0; itRed != _RLTRed.end(); ++itRed, iRow++ )
    _process_constraint( iRow, vlhs, itRed->first, itRed->second ); 
  if( options.DISPLAY >= 2 ){
    std::cout << "\nRIGHT-HAND-SIDE DEPENDENTS:\n";
    for( auto itv=_rhsRed.begin(); itv!=_rhsRed.end(); ++itv )
      std::cout << "  " << *(*itv) << std::endl;
    std::cout << "\nLEFT-HAND-SIDE NEW VARIABLES:\n";
    for( auto itv=_varRed.begin(); itv!=_varRed.end(); ++itv )
      std::cout << "  " << itv->second << ": " << *itv->first << std::endl;
    std::cout << "\nCOEFFICIENT MATRIX:\n" << _coefRed;
  }

  // Compute a basis of the left null-space using SVD
  CPPL::dcovector S;
  CPPL::dgematrix U, VT;
  if( _coefRed.dgesvd( S, U, VT ) ) return;
  if( options.DISPLAY >= 2 )
    std::cout << "\nSVD OF COEFFICIENT MATRIX:\n" << S << U;
  unsigned nCtrRed = 0;
  for( ; nCtrRed<nRLTRed; nCtrRed++ )
    if( S( nRLTRed-nCtrRed-1 ) > options.SVDTOL ) break;

  // Build "hidden" relationships
  for( unsigned i=0; i<nCtrRed; i++ ){
    FFVar ctr = U( 0, nRLTRed-i-1 ) * *_rhsRed[0];
    for( unsigned j=1; j<nRLTRed; j++ )
      ctr += U( j, nRLTRed-i-1 ) * *_rhsRed[j];
    _ctrRed.push_back( *_dag->Vars().find( &ctr ) );
  }

  // Display "hidden" relationships
  if( options.DISPLAY == 1 )
    std::cout << std::endl << nCtrRed << " REDUCED RLT CONSTRAINTS\n";
  if( options.DISPLAY >= 2 ){
    std::cout << std::endl << nCtrRed << " REDUCED RLT CONSTRAINTS FOUND:";
    for( unsigned i=0; i<nCtrRed; i++ ) std::cout << "  " << *_ctrRed[i];
    std::cout << std::endl;
    // Generate list of operation for reduction constraints
    _dag->output( _dag->subgraph( _ctrRed ) );
    //std::cout << "  " << *_ctrRed[i]
    //          << " := " << U( 0, nRLTRed-i-1 ) << " · " << *_rhsRed[0];
    //for( unsigned j=1; j<nRLTRed; j++ )
    //  std::cout << " + " << U( j, nRLTRed-i-1 ) << " · " << *_rhsRed[j];
    //std::cout << std::endl;
  }
}

inline void
RLTRED::_process_constraint
( const unsigned iRow, const std::vector<const FFVar*>& vlhs, const t_RLTVar& RLTVar, const FFOp* linOp )
{
  _rhsRed[iRow] = new FFVar(0.);

  switch( linOp->type ){
   case FFOp::PLUS:
    _process_term( iRow, RLTVar.second, *RLTVar.first, *linOp->plop, 1. );
    _process_term( iRow, RLTVar.second, *RLTVar.first, *linOp->prop, 1. );
    break;

  case FFOp::NEG:
    _process_term( iRow, RLTVar.second, *RLTVar.first, *linOp->plop, -1. );
    break;

   case FFOp::MINUS:
    _process_term( iRow, RLTVar.second, *RLTVar.first, *linOp->plop, 1. );
    _process_term( iRow, RLTVar.second, *RLTVar.first, *linOp->prop, -1. );
    break;

   case FFOp::SCALE:
    if( linOp->prop->cst() )
      _process_term( iRow, RLTVar.second, *RLTVar.first, *linOp->plop, linOp->prop->num().val() );
    else
      _process_term( iRow, RLTVar.second, *RLTVar.first, *linOp->prop, linOp->plop->num().val() );
    break;

   default:
     throw std::runtime_error( "Internal error" );
  }

  if( !_is_rhs( vlhs, *linOp->pres ) )
    _process_term( iRow, RLTVar.second, *RLTVar.first, *linOp->pres, -1. );
}

inline void
RLTRED::_process_term
( const unsigned iRow, const FFOp::TYPE& RLTOp, const FFVar& RLTVar,
  const FFVar& linVar, const double coef )
{
  bool foundTerm = false;
  switch( RLTOp ){
  // Multiplier variable
  case FFOp::TIMES:
    // Multiplied by a constant
    if( linVar.cst() ){
      *_rhsRed[iRow] += coef * linVar.num().val() * RLTVar;
      foundTerm = true;
    }
    // In a bilinear term
    for( auto itt=_bilTerms.begin(); !foundTerm && itt!=_bilTerms.end(); ++itt ){
      if( ( *(*itt)->plop == linVar && *(*itt)->prop == RLTVar )
       || ( *(*itt)->prop == linVar && *(*itt)->plop == RLTVar ) ){
        *_rhsRed[iRow] += coef * *(*itt)->pres;
        foundTerm = true;
      }
    }
    // In a fractional term
    for( auto itt=_divTerms.begin(); !foundTerm && itt!=_divTerms.end(); ++itt ){
      if( ( *(*itt)->pres == linVar && *(*itt)->prop == RLTVar )
       || ( *(*itt)->prop == linVar && *(*itt)->pres == RLTVar ) ){
        *_rhsRed[iRow] += coef * *(*itt)->plop;
        foundTerm = true;
      }
    }
    // In a square term
    for( auto itt=_sqrTerms.begin(); !foundTerm && itt!=_sqrTerms.end(); ++itt ){
      if( *(*itt)->plop == linVar && *(*itt)->plop == RLTVar ){
        *_rhsRed[iRow] += coef * *(*itt)->pres;
        foundTerm = true;
      }
    }
    // In a square-root term
    for( auto itt=_sqrtTerms.begin(); !foundTerm && itt!=_sqrtTerms.end(); ++itt ){
      if( *(*itt)->pres == linVar && *(*itt)->pres == RLTVar ){
        // Add term to rhs
        *_rhsRed[iRow] += coef * *(*itt)->plop;
        foundTerm = true;
      }
    }
    break;

  // Divider variable
  case FFOp::DIV:
    // Divided by itself
    if( linVar == RLTVar ){
      *_rhsRed[iRow] += coef;
      foundTerm = true;
    }
    // In a bilinear term
    for( auto itt=_bilTerms.begin(); !foundTerm && itt!=_bilTerms.end(); ++itt ){
      if( *(*itt)->pres == linVar && *(*itt)->plop == RLTVar ){
        *_rhsRed[iRow] += coef * *(*itt)->prop;
        foundTerm = true;
      }
    }
    // In a fractional term
    for( auto itt=_divTerms.begin(); !foundTerm && itt!=_divTerms.end(); ++itt ){
      if( *(*itt)->plop == linVar && *(*itt)->prop == RLTVar ){
        *_rhsRed[iRow] += coef * *(*itt)->pres;
        foundTerm = true;
      }
    }
    // In a square term
    for( auto itt=_sqrTerms.begin(); !foundTerm && itt!=_sqrTerms.end(); ++itt ){
      if( *(*itt)->pres == linVar && *(*itt)->plop == RLTVar ){
        *_rhsRed[iRow] += coef * *(*itt)->plop;
        foundTerm = true;
      }
    }
    // In a square-root term
    for( auto itt=_sqrtTerms.begin(); !foundTerm && itt!=_sqrtTerms.end(); ++itt ){
      if( *(*itt)->plop == linVar && *(*itt)->pres == RLTVar ){
        *_rhsRed[iRow] += coef * *(*itt)->pres;
        foundTerm = true;
      }
    }
    break;

   default:
     throw std::runtime_error( "Internal error" );
  }
  if( foundTerm ) return;

  // Insert or retreive new operation
  FFOp*newOp = new FFOp( RLTOp, const_cast<FFVar*>(&linVar), const_cast<FFVar*>(&RLTVar) );
  auto itv = _varRed.insert( std::make_pair( newOp, _varRed.size() ) );
  if( !itv.second ) delete newOp;
  const unsigned jCol = itv.first->second;
  // Assign corresponding entry in coefficent matrix
  _coefRed( iRow, jCol ) = -coef;
}

inline void
RLTRED::_extract_linearity
( const typename RLTRED::l_Ops& Ops, const std::vector<const FFVar*>& vlhs )
{
  static const unsigned int nlinOp = 4;
  static const typename FFOp::TYPE linOp[nlinOp] = { FFOp::PLUS, FFOp::NEG, FFOp::MINUS, FFOp::SCALE };

  // Create subset of linear operations
  _linTerms  = _subset_op( Ops, nlinOp, linOp );
  if( options.DISPLAY >= 2 ){
    std::cout << "\nLINEAR TERMS ACCOUNTED FOR IN RLT:\n";
    for( auto it = _linTerms.begin(); it != _linTerms.end(); ++it )
      std::cout << *(*it) << std::endl;
  }

  // Create subset of variables participating in linear operations
  // and map between these variables and their definig operations
  _subset_var_edge( vlhs, _linTerms, _linVars, _linEdges );
  if( options.DISPLAY >= 2 ){
    std::cout << "\nVARIABLES PARTICIPATING IN RLT TERMS:\n";
    for( auto it = _linVars.begin(); it != _linVars.end(); ++it )
      std::cout << *(*it) << std::endl;
    std::cout << "\nRLT TERMS <-> VARIABLES EDGES:\n";
    for( auto it = _linEdges.begin(); it != _linEdges.end(); ++it )
      std::cout << *(*it).first << "  <->  " << *(*it).second << std::endl;
  }
}

inline void
RLTRED::_target_nonlinearity
( const typename RLTRED::l_Ops& Ops )
{
  // subsets of bilinear terms
  _bilTerms  = _subset_op( Ops, FFOp::TIMES );

  // subsets of fractional terms
  _divTerms  = _subset_op( Ops, FFOp::DIV );

  // subsets of square terms
  _sqrTerms  = _subset_op( Ops, FFOp::SQR );

  // subsets of square root terms
  _sqrtTerms = _subset_op( Ops, FFOp::SQRT );
}

inline void
RLTRED::_search_sequential
( const typename RLTRED::l_Ops& Ops )
{
  _RLTRed.clear();

  // MAIN LOOP: Valid RRLT cut for each candidate multiplier/divider variables
  for( auto ito = Ops.rbegin(); ito != Ops.rend(); ++ito ){
    assert( (*ito)->pres );
    const pt_idVar& idVar = (*ito)->pres->id();

    // Multiplying/dividing constraints by a constant is pointless
    if( idVar.first == FFVar::CINT || idVar.first == FFVar::CREAL || (*ito)->pres->cst() ) continue;

    // Multiplier/divider variables limited to primary variables (no auxiliary)
    if( options.LEVEL == Options::PRIMSEQ && idVar.first == FFVar::AUX ) continue;

    // Drop any linear variable participating in a nonlinear terms multiplied by variable <a>idVar</a> and the corresponding edges
    auto VarMult = std::make_pair( (*ito)->pres, FFOp::TIMES );
    _RLTVars = _linVars;
    _RLTEdges = _linEdges;
    _bigraph_RLT( VarMult, _RLTVars, _RLTEdges );
    // Identify valid reduction constraints with the candidate multiplier variable
    _reduction_RLT( VarMult );
    //break;
 
    if( options.NODIV ) continue;
    // Drop any linear variable participating in a nonlinear terms divided by variable <a>idVar</a> and the corresponding edges
    auto VarDiv = std::make_pair( (*ito)->pres, FFOp::DIV );
    _RLTVars = _linVars;
    _RLTEdges = _linEdges;
    _bigraph_RLT( VarDiv, _RLTVars, _RLTEdges );
    // Identify valid reduction constraints with the candidate divider variable
    _reduction_RLT( VarDiv );
  }
}

inline void
RLTRED::_search_simultaneous
( const typename RLTRED::l_Ops& Ops )
{
  throw std::runtime_error( "Simultaneous RLT reduction approach not yet implemented.\n" );
}

inline void
RLTRED::_bigraph_RLT
( const t_RLTVar& VarRed, t_Vars& RLTVars, t_Edges& RLTEdges )
const
{
  // Erase those variables participating in nonlinear terms with variable <a>idVar</a>
  switch( VarRed.second ){
  case FFOp::TIMES:
    _bigraph_RLT_mult( VarRed.first, RLTVars, RLTEdges ); break;
  case FFOp::DIV:
    _bigraph_RLT_div( VarRed.first, RLTVars, RLTEdges );  break;
  default:
    throw std::runtime_error( "Internal error" );
  }
}

inline void
RLTRED::_bigraph_RLT_mult
( const FFVar*VarMult, t_Vars &linVars, t_Edges &linEdges )
const
{
  // Erase constants since their multiplication with <a>idMult</a> does not introduce a new nonlinear term
  for( auto it = linVars.begin(); it != linVars.end(); ){
    if( (*it)->id().first != FFVar::CINT && (*it)->id().first != FFVar::CREAL
     && !(*it)->cst() ) ++it;
    else{ linEdges.erase( *it ); it = linVars.erase( it ); }
  }

  // Erase those variables participating with <a>idMult</a> in bilinear terms
  for( auto it = _bilTerms.begin(); it != _bilTerms.end(); ++it ){
    if( *(*it)->plop == *VarMult ){
      linVars.erase( (*it)->prop ); linEdges.erase( (*it)->prop );
    }
    else if( *(*it)->prop == *VarMult ){
      linVars.erase( (*it)->plop ); linEdges.erase( (*it)->plop );
    }
  }

  // Erase those variables participating with <a>idMult</a> in fractional terms
  for( auto it = _divTerms.begin(); it != _divTerms.end(); ++it ){
    if( *(*it)->pres == *VarMult ){
      linVars.erase( (*it)->prop ); linEdges.erase( (*it)->prop );
    }
    else if( *(*it)->prop == *VarMult ){
      linVars.erase( (*it)->pres ); linEdges.erase( (*it)->pres );
    }
  }

  // Erase those variables participating with <a>idMult</a> in square terms
  for( auto it = _sqrTerms.begin(); it != _sqrTerms.end(); ++it ){
    if( *(*it)->plop == *VarMult ){
      linVars.erase( (*it)->plop ); linEdges.erase( (*it)->plop );
    }
  }

  // Erase those variables participating with <a>idMult</a> in square root terms
  for( auto it = _sqrtTerms.begin(); it != _sqrtTerms.end(); ++it ){
    if( *(*it)->pres == *VarMult ){
      linVars.erase( (*it)->pres ); linEdges.erase( (*it)->pres );
    }
  }

  if( options.DISPLAY >= 2 ){
    std::cout << "\nVARIABLES IN RLT TERMS NOT YET PARTICIPATING IN ANY PRODUCT TERM WITH "
              << VarMult->name() << ":\n";
    for( auto itv = linVars.begin(); itv != linVars.end(); ++itv )
      std::cout << *(*itv) << std::endl;
    std::cout << "\nBI-PARTITE GRAPH FOR : " << VarMult->name()
              << ": OPERATION <-> VARIABLE\n";
    for( auto itvo = linEdges.begin(); itvo != linEdges.end(); ++itvo )
      std::cout << *(*itvo).second << "  <->  " << *(*itvo).first << std::endl;
  }
}

inline void
RLTRED::_bigraph_RLT_div
( const FFVar* VarDiv, t_Vars &linVars, t_Edges &linEdges )
const
{
  // Erase <a>idDiv</a> since division by itself yield 1
  for( auto it = linVars.begin(); it != linVars.end(); ++it )
    if( *(*it) == *VarDiv ){ linVars.erase( *it ); break; }

  // Erase those variables participating with <a>idDiv</a> in bilinear terms
  for( auto it = _bilTerms.begin(); it != _bilTerms.end(); ++it ){
    if( *(*it)->plop == *VarDiv ){
      linVars.erase( (*it)->pres ); linEdges.erase( (*it)->pres );
    }
    else if( *(*it)->prop == *VarDiv ){
      linVars.erase( (*it)->pres ); linEdges.erase( (*it)->pres );
    }
  }

  // Erase those variables participating with <a>idDiv</a> in fractional terms
  for( auto it = _divTerms.begin(); it != _divTerms.end(); ++it ){
    if( *(*it)->pres == *VarDiv ){
      linVars.erase( (*it)->plop ); linEdges.erase( (*it)->plop );
    }
    else if( *(*it)->prop == *VarDiv ){
      linVars.erase( (*it)->plop ); linEdges.erase( (*it)->plop );
    }
  }

  // Erase those variables participating with <a>idDiv</a> in square terms
  for( auto it = _sqrTerms.begin(); it != _sqrTerms.end(); ++it ){
    if( *(*it)->plop == *VarDiv ){
      linVars.erase( (*it)->pres ); linEdges.erase( (*it)->pres );
    }
  }

  // Erase those variables participating with <a>idDiv</a> in square root terms
  for( auto it = _sqrtTerms.begin(); it != _sqrtTerms.end(); ++it ){
    if( *(*it)->pres == *VarDiv ){
      linVars.erase( (*it)->plop ); linEdges.erase( (*it)->plop );
    }
  }

  if( options.DISPLAY >= 2 ){
    std::cout << "\nVARIABLES IN RLT TERMS NOT YET PARTICIPATING IN ANY FRACTIONAL TERM WITH "
              << VarDiv->name() << ":\n";
    for( auto itv = linVars.begin(); itv != linVars.end(); ++itv )
      std::cout << *(*itv) << std::endl;
    std::cout << "\nBI-PARTITE GRAPH FOR : " << VarDiv->name()
              << ": OPERATION <-> VARIABLE\n";
    for( auto itvo = linEdges.begin(); itvo != linEdges.end(); ++itvo )
      std::cout << *(*itvo).second << "  <->  " << *(*itvo).first << std::endl;
  }
}

inline void
RLTRED::_reduction_RLT
( const t_RLTVar& VarRed )
{
  // Matching initially empty
  FFOp* NA = 0;
  _VarAssigned.assign( _RLTVars.size(), std::make_pair( NA, 0 ) );

  // Try to construct an augmenting emanating from each constraint
  auto ito = _linTerms.begin();
  for( unsigned io=0; ito != _linTerms.end(); ++ito, io++ ){
    _TermVisited.assign( _linTerms.size(), 0 );
    _VarVisited.assign( _RLTVars.size(), 0 );
    if( _augpath_RLT( std::make_pair(*ito,io) ) ) continue;

    // Append new reduction constraints
    auto jto = _linTerms.begin();
    for( unsigned jo=0; jto != _linTerms.end(); ++jto, jo++ )
      if( _TermVisited[jo] ) _RLTRed.insert( std::make_pair( VarRed, *jto ) );
  }
}

inline bool
RLTRED::_augpath_RLT
( const pt_Op& pOp )
{
  _TermVisited[pOp.second] = true;

  // Try and find immediate assignment
  auto itv = _RLTVars.begin();
  for( unsigned iv=0; itv != _RLTVars.end(); ++itv, iv++ ){
    if( _VarAssigned[iv].first ) continue;
    auto rangeOp = _RLTEdges.equal_range( *itv );
    for( auto itvo = rangeOp.first; itvo != rangeOp.second; ++itvo ){
      if( (*itvo).second != pOp.first ) continue;
      _VarAssigned[iv] = pOp;
      return true;
    }
  }

  // Try and find an augmenting path starting from another variable
  itv = _RLTVars.begin();
  for( unsigned iv=0; itv != _RLTVars.end(); ++itv, iv++ ){
    if( _VarVisited[iv] ) continue;
    auto rangeOp = _RLTEdges.equal_range( *itv );
    for( auto itvo = rangeOp.first; itvo != rangeOp.second; ++itvo ){
      if( (*itvo).second != pOp.first ) continue;
      _VarVisited[iv] = true;
      if( !_augpath_RLT( _VarAssigned[iv] ) ) continue;
      _VarAssigned[iv] = pOp;
      return true;
    }
  }

  // Failed to find an augmenting path
  return false;
}

inline typename RLTRED::t_Ops
RLTRED::_subset_op
( const typename RLTRED::l_Ops& Ops, const unsigned int nOp, const typename FFOp::TYPE*typeOp )
const
{
  // Create subset of operations of given type
  t_Ops subOps;
  for( unsigned iOp=0; iOp<nOp; iOp++ ){
    for( auto it=Ops.begin(); it!=Ops.end(); ++it ){
      if( (*it)->type != typeOp[iOp] ) continue;
      subOps.insert( *it );
    }
  }

  return subOps;
}

inline typename RLTRED::t_Ops
RLTRED::_subset_op
( const typename RLTRED::l_Ops& Ops, const typename FFOp::TYPE&typeOp )
const
{
  // Create subset of operations of given type
  t_Ops subOps;
  for( auto it=Ops.begin(); it!=Ops.end(); ++it ){
    if( (*it)->type != typeOp ) continue;
    subOps.insert( *it );
  }

  return subOps;
}

inline void
RLTRED::_subset_var_edge
( const std::vector<const FFVar*>& vlhs, const typename RLTRED::t_Ops& Ops,
  typename RLTRED::t_Vars& Vars, typename RLTRED::t_Edges& Edges )
const
{
  Vars.clear();
  Edges.clear();

  // Create subset of variables participating in a set of operations Ops
  for( auto ito = Ops.begin(); ito != Ops.end(); ++ito ){
    if( (*ito)->plop ){
      //const pt_idVar& idlop = (*ito)->plop->id();
      Vars.insert( (*ito)->plop ); // Keep constants in variable list for division
      //if( idlop.first != FFVar::CINT && idlop.first != FFVar::CREAL && !(*ito)->plop->cst() )
      Edges.insert( std::make_pair( (*ito)->plop, *ito ) );
    }
    if( (*ito)->prop ){
      //const pt_idVar& idrop = (*ito)->prop->id();
      Vars.insert( (*ito)->prop ); // Keep constants in variable list for division
      //if( idrop.first != FFVar::CINT && idrop.first != FFVar::CREAL && !(*ito)->prop->cst() )
      Edges.insert( std::make_pair( (*ito)->prop, *ito ) );
    }

    if( !(*ito)->pres || _is_rhs( vlhs, *(*ito)->pres ) ) continue;
    //bool isrhs = !(*ito)->pres;
    //for( auto itv = vlhs.begin(); !isrhs && itv != vlhs.end(); ++itv )
    //  if( **itv == *(*ito)->pres ) isrhs = true;
    //if( isrhs ) continue;
    //const pt_idVar& idres = (*ito)->pres->id();
    Vars.insert( (*ito)->pres ); // Keep constants in variable list for division
    //if( idres.first != FFVar::CINT && idres.first != FFVar::CREAL && !(*ito)->pres->cst() )
    Edges.insert( std::make_pair( (*ito)->pres, *ito ) );
  }
}

inline bool
RLTRED::_is_rhs
( const std::vector<const FFVar*>& vlhs, const FFVar& varRes )
const
{
  for( auto itv = vlhs.begin(); itv != vlhs.end(); ++itv )
    if( *(*itv) == varRes ) return true;
  return false;
}

} // namespace mc

#endif
