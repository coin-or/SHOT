#include "NLPIpoptSolver.h"

NLPIpoptSolver::NLPIpoptSolver()
{
	//IpoptSolver::IpoptSolver();
}

NLPIpoptSolver::~NLPIpoptSolver()
{
	//IpoptSolver::~IpoptSolver();
}

/*
 bool NLPIpoptSolver::intermediate_callback(AlgorithmMode mode, Index iter, Number obj_value, Number inf_pr,
 Number inf_du, Number mu, Number d_norm, Number regularization_size, Number alpha_du, Number alpha_pr,
 Index ls_trials, const IpoptData* ip_data, IpoptCalculatedQuantities* ip_cq)
 {

 Ipopt::TNLPAdapter* tnlp_adapter = NULL;

 Ipopt::OrigIpoptNLP* orignlp;
 orignlp = dynamic_cast<OrigIpoptNLP*>(GetRawPtr(ip_cq->GetIpoptNLP()));
 if (orignlp != NULL) tnlp_adapter = dynamic_cast<TNLPAdapter*>(GetRawPtr(orignlp->nlp()));
 std::cout << "HEJ!" << std::endl;
 std::exit(0);

 return false;
 }*/
