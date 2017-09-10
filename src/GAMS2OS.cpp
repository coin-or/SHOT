#include "GAMS2OS.h"

#include <cstdio>
#include <cstdlib>
#include <sys/stat.h>   // for mkdir
#include "boost/filesystem.hpp"
#include "CoinHelperFunctions.hpp" // for CoinCopyOfArrayOrZero, maybe should eliminate this
#include "ProcessInfo.h"
#include "OSoLReader.h"

// set to 3 to see gams log
#define GAMSLOGOPTION 0

GAMS2OS::GAMS2OS() :
		gmo(NULL), gev(NULL), createdtmpdir(false), osinstance(NULL), osoptions(NULL)
{
}

GAMS2OS::~GAMS2OS()
{
	clear();

	gmoLibraryUnload();
	gevLibraryUnload();
}

void GAMS2OS::readGms(const std::string& filename)
{
	char gamscall[1024];
	char buffer[GMS_SSSIZE];
	int rc;
	FILE* convertdopt;

	assert(gmo == NULL);
	assert(gev == NULL);

	/* create temporary directory */
	mkdir("loadgms.tmp", S_IRWXU);
	createdtmpdir = true;

	/* create empty convertd options file */
	convertdopt = fopen("loadgms.tmp/convertd.opt", "w");
	if (convertdopt == NULL)
	{
		throw std::logic_error("Could not create convertd options file.");
	}
	fputs(" ", convertdopt);
	fclose(convertdopt);

	/* call GAMS with convertd solver to get compiled model instance in temporary directory */
	snprintf(gamscall, sizeof(gamscall), GAMSDIR "/gams %s SOLVER=CONVERTD SCRDIR=loadgms.tmp output=loadgms.tmp/listing optdir=loadgms.tmp optfile=1 pf4=0 solprint=0 limcol=0 limrow=0 pc=2 lo=%d", filename.c_str(), GAMSLOGOPTION);
	/* printf(gamscall); fflush(stdout); */
	rc = system(gamscall);
	if (rc != 0)
	{
		snprintf(buffer, sizeof(buffer), "GAMS call returned with code %d", rc);
		throw std::logic_error(buffer);
	}

	readCntr("loadgms.tmp/gamscntr.dat");
}

void GAMS2OS::readCntr(const std::string& filename)
{
	char buffer[GMS_SSSIZE];

	/* initialize GMO and GEV libraries */
	if (!gmoCreateDD(&gmo, GAMSDIR, buffer, sizeof(buffer)) || !gevCreateDD(&gev, GAMSDIR, buffer, sizeof(buffer))) throw std::logic_error(
			buffer);

	/* load control file */
	if (gevInitEnvironmentLegacy(gev, filename.c_str()))
	{
		gmoFree (&gmo);
		gevFree (&gev);
		throw std::logic_error("Could not load control file loadgms.tmp/gamscntr.dat.");
	}

	if (gmoRegisterEnvironment(gmo, gev, buffer))
	{
		gmoFree (&gmo);
		gevFree (&gev);
		snprintf(buffer, sizeof(buffer), "Error registering GAMS Environment: %s", buffer);
		throw std::logic_error(buffer);
	}

	if (gmoLoadDataLegacy(gmo, buffer))
	{
		gmoFree (&gmo);
		gevFree (&gev);
		throw std::logic_error("Could not load model data.");
	}
}

void GAMS2OS::createOSObjects()
{
	char buffer[GMS_SSSIZE];

	// read options file, if specified; assuming OSoL format
	delete osoptions;
	osoptions = NULL;

	if (gmoOptFile(gmo) > 0)
	{
		gmoNameOptFile(gmo, buffer);
		if (!boost::filesystem::exists(buffer)) throw std::logic_error("Options file not found.");

		gevLogPChar(gev, "Reading options from ");
		gevLog(gev, buffer);

		OSoLReader* osolreader = new OSoLReader();
		FileUtil *fileUtil = new FileUtil();

		std::string fileContents = fileUtil->getFileAsString(buffer);
		osoptions = osolreader->readOSoL(fileContents);

		delete fileUtil;
		delete osolreader;
	}

	/* reformulate objective variable out of model, if possible */
	gmoObjReformSet(gmo, 1);
	gmoObjStyleSet(gmo, gmoObjType_Fun);
	gmoMinfSet(gmo, -OSDBL_MAX);
	gmoPinfSet(gmo, OSDBL_MAX);
	gmoIndexBaseSet(gmo, 0);
	gmoUseQSet(gmo, 1);

	// delete possible old instance and create a new one
	delete osinstance;
	osinstance = new GAMSOSInstance(gmo);

	gmoNameInput(gmo, buffer);
	osinstance->setInstanceName(buffer);

	// setup variables
	osinstance->setVariableNumber(gmoN(gmo));

	char* vartypes = new char[gmoN(gmo)];
	std::string* varnames = new std::string[gmoN(gmo)];
	for (int i = 0; i < gmoN(gmo); ++i)
	{
		switch (gmoGetVarTypeOne(gmo, i))
		{
			case gmovar_X:
				vartypes[i] = 'C';
				break;
			case gmovar_B:
				vartypes[i] = 'B';
				break;
			case gmovar_I:
				vartypes[i] = 'I';
				break;
			case gmovar_SC:
				vartypes[i] = 'D';
				break;
			case gmovar_SI:
				vartypes[i] = 'J';
				break;
			case gmovar_S1:
			case gmovar_S2:
				vartypes[i] = 'C';
				// ignore SOS1, SOS2 for now; we should only get here if called by convert, in which case we do something special
				break;
			default:
			{
				throw std::logic_error("Unsupported variable type.");
			}
		}
		if (gmoDict (gmo)) gmoGetVarNameOne(gmo, i, buffer);
		else sprintf(buffer, "x%08d", i);
		varnames[i] = buffer;
	}

	double* varlow = new double[gmoN(gmo)];
	double* varup = new double[gmoN(gmo)];
	gmoGetVarLower(gmo, varlow);
	gmoGetVarUpper(gmo, varup);

	if (gmoN(gmo) > 0 && !osinstance->setVariables(gmoN(gmo), varnames, varlow, varup, vartypes)) throw std::logic_error(
			"OSInstance::setVariables did not succeed.");

	delete[] vartypes;
	delete[] varnames;
	delete[] varlow;
	delete[] varup;

	// setup linear part of objective, if any
	if (gmoModelType(gmo) == gmoProc_cns)
	{
		// no objective in constraint satisfaction models
		osinstance->setObjectiveNumber(0);
	}
	else
	{
		osinstance->setObjectiveNumber(1);

		SparseVector* objectiveCoefficients = NULL;

		if (gmoN(gmo) > 0)
		{
			int* colidx = new int[gmoObjNZ(gmo)];
			double* val = new double[gmoObjNZ(gmo)];
			int* nlflag = new int[gmoObjNZ(gmo)];
			int nz;
			int nlnz;

			gmoGetObjSparse(gmo, colidx, val, nlflag, &nz, &nlnz);

			objectiveCoefficients = new SparseVector(nz - nlnz);
			for (int i = 0, j = 0; i < nz; ++i)
			{
				if (nlflag[i]) continue;
				objectiveCoefficients->indexes[j] = colidx[i];
				objectiveCoefficients->values[j] = val[i];
				j++;
				assert(j <= nz - nlnz);
			}

			delete[] colidx;
			delete[] val;
			delete[] nlflag;
		}
		else
		{
			objectiveCoefficients = new SparseVector(0);
		}

		if (gmoDict (gmo)) gmoGetObjName(gmo, buffer);
		else strcpy(buffer, "objective");

		if (!osinstance->addObjective(-1, buffer, gmoSense(gmo) == gmoObj_Min ? "min" : "max", gmoObjConst(gmo), 1.0,
				objectiveCoefficients))
		{
			delete objectiveCoefficients;
			throw std::logic_error("OSInstance::addObjective did not succeed.");
		}
		delete objectiveCoefficients;
	}

	// setup linear part of constraints
	osinstance->setConstraintNumber(gmoM(gmo));

	double lb;
	double ub;
	for (int i = 0; i < gmoM(gmo); ++i)
	{
		switch (gmoGetEquTypeOne(gmo, i))
		{
			case gmoequ_E:
				lb = ub = gmoGetRhsOne(gmo, i);
				break;

			case gmoequ_L:
				lb = -OSDBL_MAX;
				ub = gmoGetRhsOne(gmo, i);
				break;

			case gmoequ_G:
				lb = gmoGetRhsOne(gmo, i);
				ub = OSDBL_MAX;
				break;

			case gmoequ_N:
				lb = -OSDBL_MAX;
				ub = OSDBL_MAX;
				break;

			default:
				throw std::logic_error("Unknown row type.");
		}

		if (gmoDict (gmo)) gmoGetEquNameOne(gmo, i, buffer);
		else sprintf(buffer, "e%08d", i);

		if (!osinstance->addConstraint(i, buffer, lb, ub, 0.0))
		{
			throw std::logic_error("OSInstance::addConstraint did not succeed.");
		}
	}

	double* values = new double[gmoNZ(gmo) + gmoN(gmo)];
	int* colindexes = new int[gmoNZ(gmo) + gmoN(gmo)];
	int* rowstarts = new int[gmoM(gmo) + 1];
	int* nlflags = new int[gmoN(gmo)];

	int nz = 0;
	for (int row = 0; row < gmoM(gmo); ++row)
	{
		int rownz;
		int nlnz;

		rowstarts[row] = nz;
		gmoGetRowSparse(gmo, row, &colindexes[nz], &values[nz], nlflags, &rownz, &nlnz);

		if (nlnz > 0)
		{
			/* remove coefs of nonlinear entries (strangely, they seem to be disappearing for general nonlinear rows anyway when enabling qmaker...) */
			for (int k = 0, shift = 0; k + shift < rownz;)
			{
				if (nlflags[k + shift])
				{
					++shift;
				}
				else
				{
					colindexes[nz + k] = colindexes[nz + k + shift];
					values[nz + k] = values[nz + k + shift];
					++k;
				}
			}
		}
		nz += rownz - nlnz;
	}
	rowstarts[gmoM(gmo)] = nz;

	// values, rowstarts, colindexes are deleted by OSInstance
	delete[] nlflags;

	if (!osinstance->setLinearConstraintCoefficients(nz, false, values, 0, nz - 1, colindexes, 0, nz - 1, rowstarts, 0,
			gmoM(gmo))) throw std::logic_error("OSInstance::setLinearConstraintCoefficients did not succeed.");

	// if everything linear, then we're finished
	if (gmoObjNLNZ(gmo) == 0 && gmoNLNZ(gmo) == 0) return;

	// setup quadratic terms and nonlinear expressions
	int nqterms = 0;
	if (gmoGetObjOrder(gmo) == gmoorder_Q) nqterms = gmoObjQNZ(gmo);
	for (int i = 0; i < gmoM(gmo); ++i)
		if (gmoGetEquOrderOne(gmo, i) == gmoorder_Q) nqterms += gmoGetRowQNZOne(gmo, i);

	int qtermpos = 0;
	int* quadequs = new int[nqterms];
	int* quadrows = new int[nqterms];
	int* quadcols = new int[nqterms];
	double* quadcoefs = new double[nqterms];

	if (osinstance->instanceData->nonlinearExpressions == NULL) osinstance->instanceData->nonlinearExpressions =
			new NonlinearExpressions();

	osinstance->instanceData->nonlinearExpressions->numberOfNonlinearExpressions = gmoNLM(gmo)
			+ (gmoGetObjOrder(gmo) == gmoorder_NL ? 1 : 0);
	osinstance->instanceData->nonlinearExpressions->nl = CoinCopyOfArrayOrZero((Nl**) NULL,
			osinstance->instanceData->nonlinearExpressions->numberOfNonlinearExpressions);
	int iNLidx = 0;

	int* opcodes = new int[gmoNLCodeSizeMaxRow(gmo) + 1];
	int* fields = new int[gmoNLCodeSizeMaxRow(gmo) + 1];
	int constantlen = gmoNLConst(gmo);
	double* constants = (double*) gmoPPool(gmo);
	int codelen;
	OSnLNode* nl;

	if (gmoGetObjOrder(gmo) == gmoorder_Q)
	{
		// handle quadratic objective
		assert(qtermpos == 0);

		int qnz = gmoObjQNZ(gmo);
		gmoGetObjQ(gmo, quadrows, quadcols, quadcoefs);
		for (int j = 0; j < qnz; ++j)
		{
			if (quadrows[j] == quadcols[j]) quadcoefs[j] /= 2.0; /* for some strange reason, the coefficients on the diagonal are multiplied by 2 in GMO */
			quadequs[j] = -1;
		}
		qtermpos = qnz;
	}
	else if (gmoObjNLNZ(gmo) > 0)
	{
		// handle nonlinear objective
		gmoDirtyGetObjFNLInstr(gmo, &codelen, opcodes, fields);

		nl = parseGamsInstructions(codelen, opcodes, fields, constantlen, constants);
		if (nl == NULL)
		{
			sprintf(buffer, "Failure when processing GAMS instructions for objective %s.",
					osinstance->getObjectiveNames()[0].c_str());
			throw std::logic_error(buffer);
		}

		double objjacval = gmoObjJacVal(gmo);
		if (objjacval == 1.0)
		{
			// scale by -1/objjacval = negate
			OSnLNode* negnode = new OSnLNodeNegate;
			negnode->m_mChildren[0] = nl;
			nl = negnode;
		}
		else if (objjacval != -1.0)
		{
			// scale by -1/objjacval
			OSnLNodeNumber* numbernode = new OSnLNodeNumber();
			numbernode->value = -1 / objjacval;
			OSnLNodeTimes* timesnode = new OSnLNodeTimes();
			timesnode->m_mChildren[0] = nl;
			timesnode->m_mChildren[1] = numbernode;
			nl = timesnode;
		}

		assert(iNLidx < osinstance->instanceData->nonlinearExpressions->numberOfNonlinearExpressions);
		osinstance->instanceData->nonlinearExpressions->nl[iNLidx] = new Nl();
		osinstance->instanceData->nonlinearExpressions->nl[iNLidx]->idx = -1;
		osinstance->instanceData->nonlinearExpressions->nl[iNLidx]->osExpressionTree = new ScalarExpressionTree();
		osinstance->instanceData->nonlinearExpressions->nl[iNLidx]->osExpressionTree->m_treeRoot = nl;
		++iNLidx;
	}

	for (int i = 0; i < gmoM(gmo); ++i)
	{
		if (gmoGetEquOrderOne(gmo, i) == gmoorder_Q)
		{
			// handle quadratic equation
			int qnz = gmoGetRowQNZOne(gmo, i);
			assert(qtermpos + qnz <= nqterms);
			gmoGetRowQ(gmo, i, &quadrows[qtermpos], &quadcols[qtermpos], &quadcoefs[qtermpos]);
			for (int j = qtermpos; j < qtermpos + qnz; ++j)
			{
				if (quadrows[j] == quadcols[j]) quadcoefs[j] /= 2.0; /* for some strange reason, the coefficients on the diagonal are multiplied by 2 in GMO */
				quadequs[j] = i;
			}
			qtermpos += qnz;
		}
		else if (gmoGetEquOrderOne(gmo, i) == gmoorder_NL)
		{
			gmoDirtyGetRowFNLInstr(gmo, i, &codelen, opcodes, fields);
			if (codelen == 0) continue;

			nl = parseGamsInstructions(codelen, opcodes, fields, constantlen, constants);
			if (nl == NULL)
			{
				sprintf(buffer, "Failure when processing GAMS instructions for equation %s.",
						osinstance->getConstraintNames()[i].c_str());
				throw std::logic_error(buffer);
			}

			assert(iNLidx < osinstance->instanceData->nonlinearExpressions->numberOfNonlinearExpressions);
			osinstance->instanceData->nonlinearExpressions->nl[iNLidx] = new Nl();
			osinstance->instanceData->nonlinearExpressions->nl[iNLidx]->idx = i; // correct that this is the con. number?
			osinstance->instanceData->nonlinearExpressions->nl[iNLidx]->osExpressionTree = new ScalarExpressionTree();
			osinstance->instanceData->nonlinearExpressions->nl[iNLidx]->osExpressionTree->m_treeRoot = nl;
			++iNLidx;
		}
	}

	assert(qtermpos == nqterms);
	assert(iNLidx == osinstance->instanceData->nonlinearExpressions->numberOfNonlinearExpressions);

	osinstance->setQuadraticCoefficients(nqterms, quadequs, quadrows, quadcols, quadcoefs, 0, nqterms - 1);

	delete[] quadequs;
	delete[] quadrows;
	delete[] quadcols;
	delete[] quadcoefs;
	delete[] opcodes;
	delete[] fields;
}

const char* multivarops[] =
{ "sum", "product", "min", "max", "allDiff" };

static
void applyOperation(std::vector<OSnLNode*>& stack, OSnLNode* op, int nargs = -1)
{
	assert(op != NULL);

	if (op->getTokenName() == "minus")
	{
		/* for A - B with A a sum, rewrite as A + (-B), so we can add -B into the sum of A */
		assert(nargs == 2 || nargs == -1);
		assert(stack.size() >= 2);
		if (stack[stack.size() - 2]->getTokenName() == "sum")
		{
			delete op;
			applyOperation(stack, new OSnLNodeNegate());
			applyOperation(stack, new OSnLNodeSum(), 2);
			return;
		}
	}

	if (nargs < 0) nargs = op->inumberOfChildren;

	/* if operator can take arbitrarily many operands, then merge children of children of same operator into new one
	 * e.g., (a+b) + c = a + b + c */
	for (int o = 0; o < 4; ++o)
	{
		if (op->getTokenName() != multivarops[o]) continue;

		assert(op->inumberOfChildren == 0);
		assert(op->m_mChildren == NULL);
		assert(nargs > 0);
		assert((int) stack.size() >= nargs);

		op->inumberOfChildren = nargs;

		/* check for number of additional arguments from arguments of same operator */
		size_t a;
		for (a = 0; (int) a < nargs; ++a)
			if (stack[stack.size() - 1 - a]->getTokenName() == multivarops[o]) op->inumberOfChildren +=
					stack[stack.size() - 1 - a]->inumberOfChildren - 1;

		op->m_mChildren = new OSnLNode*[op->inumberOfChildren];

		a = 0;
		while (nargs)
		{
			if (stack.back()->getTokenName() == multivarops[o])
			{
				assert(a + stack.back()->inumberOfChildren <= op->inumberOfChildren);
				memcpy(&op->m_mChildren[a], stack.back()->m_mChildren,
						stack.back()->inumberOfChildren * sizeof(OSnLNode*));
				a += stack.back()->inumberOfChildren;

				delete[] stack.back()->m_mChildren;
				stack.back()->m_mChildren = NULL;
				stack.back()->inumberOfChildren = 0;
				delete stack.back();
			}
			else
			{
				assert(a + 1 <= op->inumberOfChildren);
				op->m_mChildren[a] = stack.back();
				++a;
			}
			stack.pop_back();
			--nargs;
		}
		assert(a == op->inumberOfChildren);

		stack.push_back(op);
		return;
	}

	if (op->inumberOfChildren == 0 && nargs > 0)
	{
		assert(op->m_mChildren == NULL);

		op->inumberOfChildren = nargs;
		op->m_mChildren = new OSnLNode*[nargs];
	}
	assert((int) op->inumberOfChildren == nargs);
	assert(op->m_mChildren != NULL || nargs == 0);

	assert((int) stack.size() >= nargs);

	while (nargs)
	{
		op->m_mChildren[--nargs] = stack.back();
		stack.pop_back();
	}

	stack.push_back(op);
}

OSnLNode* GAMS2OS::parseGamsInstructions(int codelen, /**< length of GAMS instructions */
int* opcodes, /**< opcodes of GAMS instructions */
int* fields, /**< fields of GAMS instructions */
int constantlen, /**< length of GAMS constants pool */
double* constants /**< GAMS constants pool */
)
{
	bool debugoutput = gevGetIntOpt(gev, gevInteger1) & 0x4;
#define debugout if( debugoutput ) std::clog

	std::vector<OSnLNode*> stack;
	stack.reserve(20);

	int nargs = -1;

	for (int i = 0; i < codelen; ++i)
	{
		GamsOpCode opcode = (GamsOpCode) opcodes[i];
		int address = fields[i] - 1;

		debugout << '\t' << GamsOpCodeName[opcode] << ": ";
		switch (opcode)
		{
			case nlNoOp:   // no operation
			case nlStore:  // store row
			case nlHeader: // header
			{
				debugout << "ignored" << std::endl;
				break;
			}

			case nlPushV: // push variable
			{
				address = gmoGetjSolver(gmo, address);
				debugout << "push variable " << osinstance->getVariableNames()[address] << std::endl;
				OSnLNodeVariable* nlNode = new OSnLNodeVariable();
				nlNode->idx = address;
				stack.push_back(nlNode);
				break;
			}

			case nlPushI: // push constant
			{
				debugout << "push constant " << constants[address] << std::endl;
				OSnLNodeNumber* nlNode = new OSnLNodeNumber();
				nlNode->value = constants[address];
				stack.push_back(nlNode);
				break;
			}

			case nlPushZero: // push zero
			{
				debugout << "push constant zero" << std::endl;
				stack.push_back(new OSnLNodeNumber());
				break;
			}

			case nlAdd: // add
			{
				debugout << "add" << std::endl;

				applyOperation(stack, new OSnLNodeSum(), 2);

				break;
			}

			case nlAddV: // add variable
			{
				address = gmoGetjSolver(gmo, address);
				debugout << "add variable " << osinstance->getVariableNames()[address] << std::endl;
				OSnLNodeVariable* nlNode = new OSnLNodeVariable();
				nlNode->idx = address;
				stack.push_back(nlNode);
				applyOperation(stack, new OSnLNodeSum(), 2);
				break;
			}

			case nlAddI: // add immediate
			{
				debugout << "add constant " << constants[address] << std::endl;
				OSnLNodeNumber* nlNode = new OSnLNodeNumber();
				nlNode->value = constants[address];
				stack.push_back(nlNode);
				applyOperation(stack, new OSnLNodeSum(), 2);
				break;
			}

			case nlSub: // minus
			{
				debugout << "minus" << std::endl;
				applyOperation(stack, new OSnLNodeMinus());
				break;
			}

			case nlSubV: // subtract variable
			{
				address = gmoGetjSolver(gmo, address);
				debugout << "subtract variable " << osinstance->getVariableNames()[address] << std::endl;
				OSnLNodeVariable* nlNode = new OSnLNodeVariable();
				nlNode->idx = address;
				stack.push_back(nlNode);
				applyOperation(stack, new OSnLNodeMinus());
				break;
			}

			case nlSubI: // subtract immediate
			{
				debugout << "subtract constant " << constants[address] << std::endl;
				OSnLNodeNumber* nlNode = new OSnLNodeNumber();
				nlNode->value = constants[address];
				stack.push_back(nlNode);
				applyOperation(stack, new OSnLNodeMinus());
				break;
			}

			case nlMul: // multiply
			{
				debugout << "multiply" << std::endl;
				applyOperation(stack, new OSnLNodeProduct(), 2);
				break;
			}

			case nlMulV: // multiply variable
			{
				address = gmoGetjSolver(gmo, address);
				debugout << "multiply variable " << osinstance->getVariableNames()[address] << std::endl;
				OSnLNodeVariable* nlNode = new OSnLNodeVariable();
				nlNode->idx = address;
				stack.push_back(nlNode);
				applyOperation(stack, new OSnLNodeProduct(), 2);
				break;
			}

			case nlMulI: // multiply immediate
			{
				debugout << "multiply constant " << constants[address] << std::endl;
				OSnLNodeNumber* nlNode = new OSnLNodeNumber();
				nlNode->value = constants[address];
				stack.push_back(nlNode);
				applyOperation(stack, new OSnLNodeProduct(), 2);
				break;
			}

			case nlMulIAdd: // multiply immediate and add
			{
				debugout << "multiply constant " << constants[address] << " and add " << std::endl;
				OSnLNodeNumber* nlNode = new OSnLNodeNumber();
				nlNode->value = constants[address];
				stack.push_back(nlNode);
				applyOperation(stack, new OSnLNodeProduct(), 2);
				applyOperation(stack, new OSnLNodeSum(), 2);
				break;
			}

			case nlDiv: // divide
			{
				debugout << "divide" << std::endl;
				applyOperation(stack, new OSnLNodeDivide());
				break;
			}

			case nlDivV: // divide variable
			{
				address = gmoGetjSolver(gmo, address);
				debugout << "divide variable " << osinstance->getVariableNames()[address] << std::endl;
				OSnLNodeVariable* nlNode = new OSnLNodeVariable();
				nlNode->idx = address;
				stack.push_back(nlNode);
				applyOperation(stack, new OSnLNodeDivide());
				break;
			}

			case nlDivI: // divide immediate
			{
				debugout << "divide constant " << constants[address] << std::endl;
				OSnLNodeNumber* nlNode = new OSnLNodeNumber();
				nlNode->value = constants[address];
				stack.push_back(nlNode);
				applyOperation(stack, new OSnLNodeDivide());
				break;
			}

			case nlUMin: // unary minus
			{
				debugout << "negate" << std::endl;
				applyOperation(stack, new OSnLNodeNegate());
				break;
			}

			case nlUMinV: // unary minus variable
			{
				address = gmoGetjSolver(gmo, address);
				debugout << "push negated variable " << osinstance->getVariableNames()[address] << std::endl;
				OSnLNodeVariable* nlNode = new OSnLNodeVariable();
				nlNode->idx = address;
				nlNode->coef = -1.0;
				stack.push_back(nlNode);
				break;
			}

			case nlFuncArgN: // number of function arguments
			{
				nargs = address;
				debugout << nargs << "arguments" << std::endl;
				break;
			}

			case nlCallArg1:
			case nlCallArg2:
			case nlCallArgN:
			{
				debugout << "call function ";

				switch (GamsFuncCode(address + 1))
				// undo shift by 1
				{
					case fnmin:
					{
						debugout << "min" << std::endl;
						applyOperation(stack, new OSnLNodeMin(), 2);
						break;
					}

					case fnmax:
					{
						debugout << "max" << std::endl;
						applyOperation(stack, new OSnLNodeMax(), 2);
						break;
					}

					case fnsqr:
					{
						debugout << "square" << std::endl;
						applyOperation(stack, new OSnLNodeSquare());
						break;
					}

					case fnexp:
					case fnslexp:
					case fnsqexp:
					{
						debugout << "exp" << std::endl;
						applyOperation(stack, new OSnLNodeExp());
						break;
					}

					case fnlog:
					{
						debugout << "ln" << std::endl;
						applyOperation(stack, new OSnLNodeLn());
						break;
					}

					case fnlog10:
					case fnsllog10:
					case fnsqlog10:
					{
						debugout << "log10 = ln * 1/ln(10)" << std::endl;
						applyOperation(stack, new OSnLNodeLn());
						OSnLNodeNumber* nlNode = new OSnLNodeNumber();
						nlNode->value = 1.0 / log(10.0);
						stack.push_back(nlNode);
						applyOperation(stack, new OSnLNodeTimes());
						break;
					}

					case fnlog2:
					{
						debugout << "log2 = ln * 1/ln(2)" << std::endl;
						applyOperation(stack, new OSnLNodeLn());
						OSnLNodeNumber *nlNode = new OSnLNodeNumber();
						nlNode->value = 1.0 / log(2.0);
						stack.push_back(nlNode);
						applyOperation(stack, new OSnLNodeTimes());
						break;
					}

					case fnsqrt:
					{
						debugout << "sqrt" << std::endl;
						applyOperation(stack, new OSnLNodeSqrt());
						break;
					}

					case fnabs:
					{
						debugout << "abs" << std::endl;
						applyOperation(stack, new OSnLNodeAbs());
						break;
					}

					case fncos:
					{
						debugout << "cos" << std::endl;
						applyOperation(stack, new OSnLNodeCos());
						break;
					}

					case fnsin:
					{
						debugout << "sin" << std::endl;
						applyOperation(stack, new OSnLNodeSin());
						break;
					}

					case fnpower:
					case fnrpower: // x ^ y
					case fncvpower: // constant ^ x
					case fnvcpower: // x ^ constant
					{
						debugout << "power" << std::endl;
						applyOperation(stack, new OSnLNodePower());
						break;
					}

					case fnpi:
					{
						debugout << "pi" << std::endl;
						stack.push_back(new OSnLNodePI());
						break;
					}

					case fndiv:
					case fndiv0:
					{
						debugout << "divide" << std::endl;
						applyOperation(stack, new OSnLNodeDivide());
						break;
					}

					case fnslrec: // 1/x
					case fnsqrec: // 1/x
					{
						debugout << "reciprocal" << std::endl;
						OSnLNodeNumber *nlNode = new OSnLNodeNumber();
						nlNode->value = 1.0;
						stack.push_back(nlNode);
						applyOperation(stack, new OSnLNodeDivide());
						break;
					}

						/* cannot handle nonlinear polynomials with this setup, as we would require copies of
						 * the variable argument when the variable occurs more than once
						 */
#if 0
						case fnpoly: // simple polynomial
						{
							debugout << "polynomial of degree " << nargs-1 << std::endl;
							assert(nargs >= 0);
							switch( nargs )
							{
								case 0:
								{
									// delete variable of polynom
									delete stack.back();
									stack.pop_back();
									// push zero
									stack.push_back(new OSnLNodeNumber());
									break;
								}

								case 1: // "constant" polynomial
								{
									assert(stack.size() >= 2);
									// delete variable
									delete stack[stack.size()-2];
									// put constant there
									stack[stack.size()-2] = stack.back();
									// forget last element
									stack.pop_back();
									break;
								}

								default: // polynomial is at least linear
								{
									std::vector<OSnLNode*> coeff(nargs);
									while( nargs > 0 )
									{
										assert(!stack.empty());
										coeff[nargs-1] = stack.back();
										stack.pop_back();
										--nargs;
									}
									assert(!stack.empty());
									OSnLNode* var = stack.back();
									stack.pop_back();

									stack.push_back(coeff[0]);

									stack.push_back(coeff[1]);
									stack.push_back(var);
									applyOperation(stack, new OSnLNodeProduct(), 2);

									for( size_t i = 2; i < coeff.size(); ++i )
									{
										stack.push_back(coeff[i]);
										stack.push_back(var->copyNodeAndDescendants()); // TODO this isn't sufficient in cases where var is not just a variable or expression)
										if( i == 2 )
										{
											applyOperation(stack, new OSnLNodeSquare());
										}
										else
										{
											OSnLNodeNumber* exponent = new OSnLNodeNumber();
											exponent->value = (double)i;
											stack.push_back(exponent);
											applyOperation(stack, new OSnLNodePower());
										}
										applyOperation(stack, new OSnLNodeProduct(), 2);
									}

									applyOperation(stack, new OSnLNodeSum(), coeff.size());
								}
							}
							nargs = -1;
							break;
						}
#endif
					case fnerrf:
					{
						debugout << "errorf = 0.5 * [1+erf(x/sqrt(2))]" << std::endl;
						OSnLNodeNumber* nodenumber;

						nodenumber = new OSnLNodeNumber();
						nodenumber->value = sqrt(2.0) / 2.0;
						stack.push_back(nodenumber);
						applyOperation(stack, new OSnLNodeProduct(), 2);

						applyOperation(stack, new OSnLNodeErf());

						nodenumber = new OSnLNodeNumber();
						nodenumber->value = 1.0;
						stack.push_back(nodenumber);
						applyOperation(stack, new OSnLNodePlus());

						nodenumber = new OSnLNodeNumber();
						nodenumber->value = 0.5;
						stack.push_back(nodenumber);
						applyOperation(stack, new OSnLNodeTimes());

						break;
					}

						// TODO some more we could handle
					case fnceil:
					case fnfloor:
					case fnround:
					case fnmod:
					case fntrunc:
					case fnsign:
					case fnarctan:
					case fndunfm:
					case fndnorm:
					case fnerror:
					case fnfrac:
					case fnerrorl:
					case fnfact /* factorial */:
					case fnunfmi /* uniform random number */:
					case fnncpf /* fischer: sqrt(x1^2+x2^2+2*x3) */:
					case fnncpcm /* chen-mangasarian: x1-x3*ln(1+exp((x1-x2)/x3))*/:
					case fnentropy /* x*ln(x) */:
					case fnsigmoid /* 1/(1+exp(-x)) */:
					case fnboolnot:
					case fnbooland:
					case fnboolor:
					case fnboolxor:
					case fnboolimp:
					case fnbooleqv:
					case fnrelopeq:
					case fnrelopgt:
					case fnrelopge:
					case fnreloplt:
					case fnrelople:
					case fnrelopne:
					case fnifthen:
					case fnedist /* euclidian distance */:
					case fncentropy /* x*ln((x+d)/(y+d))*/:
					case fngamma:
					case fnloggamma:
					case fnbeta:
					case fnlogbeta:
					case fngammareg:
					case fnbetareg:
					case fnsinh:
					case fncosh:
					case fntanh:
					case fnsignpower /* sign(x)*abs(x)^c */:
					case fnncpvusin /* veelken-ulbrich */:
					case fnncpvupow /* veelken-ulbrich */:
					case fnbinomial:
					case fntan:
					case fnarccos:
					case fnarcsin:
					case fnarctan2 /* arctan(x2/x1) */:
					case fnpoly:
					default:
					{
						debugout << "nr. " << address + 1 << " - unsuppored. Error." << std::endl;
						char buffer[256];
						sprintf(buffer, "Error: Unsupported GAMS function %s.\n", GamsFuncCodeName[address + 1]);
						gevLogStatPChar(gev, buffer);
						while (!stack.empty())
						{
							delete stack.back();
							stack.pop_back();
						}
						return NULL;
					}
				}
				break;
			}

			default:
			{
				debugout << "opcode " << opcode << " - unsuppored. Error." << std::endl;
				char buffer[256];
				sprintf(buffer, "Error: Unsupported GAMS opcode %s.\n", GamsOpCodeName[opcode]);
				gevLogStatPChar(gev, buffer);
				while (!stack.empty())
				{
					delete stack.back();
					stack.pop_back();
				}
				return NULL;
			}
		}
	}

	assert(stack.size() == 1);

	return stack[0];
#undef debugout
}

void GAMS2OS::writeResult(OSResult& osresult)
{
#if 0
	if( osresult.general == NULL )
	{
		gmoModelStatSet(gmo, gmoModelStat_ErrorNoSolution);
		gmoSolveStatSet(gmo, gmoSolveStat_SolverErr);
		gevLogStat(gev, "Error: OS result does not have header.");
		return;
	}
	else if( osresult.getGeneralStatusType() == "error" )
	{
		gmoModelStatSet(gmo, gmoModelStat_ErrorNoSolution);
		gmoSolveStatSet(gmo, gmoSolveStat_SolverErr);
		gevLogStatPChar(gev, "Error: OS result reports error: ");
		gevLogStatPChar(gev, osresult.getGeneralMessage().c_str());
		gevLogStat(gev, "");
		return;
	}
	else if( osresult.getGeneralStatusType() == "warning" )
	{
		gevLogStatPChar(gev, "Warning: OS result reports warning: ");
		gevLogStatPChar(gev, osresult.getGeneralMessage().c_str());
		gevLogStat(gev, "");
	}
#endif

	gmoSolveStatSet(gmo, gmoSolveStat_Normal);

	if (osresult.getSolutionNumber() == 0) gmoModelStatSet(gmo, gmoModelStat_NoSolutionReturned);
	else if (osresult.getSolutionStatusType(0) == "unbounded") gmoModelStatSet(gmo, gmoModelStat_Unbounded);
	else if (osresult.getSolutionStatusType(0) == "globallyOptimal") gmoModelStatSet(gmo, gmoModelStat_OptimalGlobal);
	else if (osresult.getSolutionStatusType(0) == "locallyOptimal") gmoModelStatSet(gmo, gmoModelStat_OptimalLocal);
	else if (osresult.getSolutionStatusType(0) == "optimal") gmoModelStatSet(gmo, gmoModelStat_OptimalGlobal);
	else if (osresult.getSolutionStatusType(0) == "bestSoFar") gmoModelStatSet(gmo, gmoModelStat_Feasible); // TODO report integer solution if integer var.?
	else if (osresult.getSolutionStatusType(0) == "feasible") gmoModelStatSet(gmo, gmoModelStat_Feasible); // TODO report integer solution if integer var.?
	else if (osresult.getSolutionStatusType(0) == "infeasible") gmoModelStatSet(gmo, gmoModelStat_InfeasibleGlobal);
	else if (osresult.getSolutionStatusType(0) == "stoppedByLimit")
	{
		gmoSolveStatSet(gmo, gmoSolveStat_Resource); // just a guess
		gmoModelStatSet(gmo, gmoModelStat_InfeasibleIntermed);
	}
	else if (osresult.getSolutionStatusType(0) == "unsure") gmoModelStatSet(gmo, gmoModelStat_InfeasibleIntermed);
	else if (osresult.getSolutionStatusType(0) == "error") gmoModelStatSet(gmo, gmoModelStat_ErrorUnknown);
	else if (osresult.getSolutionStatusType(0) == "other") gmoModelStatSet(gmo, gmoModelStat_InfeasibleIntermed);
	else gmoModelStatSet(gmo, gmoModelStat_ErrorUnknown);

	if (osresult.getVariableNumber() != gmoN(gmo))
	{
		gevLogStat(gev, "Error: Number of variables in OS result does not match with gams model.");
		gmoModelStatSet(gmo, gmoModelStat_ErrorNoSolution);
		gmoSolveStatSet(gmo, gmoSolveStat_SystemErr);
		return;
	}
	if (osresult.getConstraintNumber() != gmoM(gmo))
	{
		gevLogStat(gev, "Error: Number of constraints in OS result does not match with gams model.");
		gmoModelStatSet(gmo, gmoModelStat_ErrorNoSolution);
		gmoSolveStatSet(gmo, gmoSolveStat_SystemErr);
		return;
	}

	// TODO some more statistics we can get from OSrL?
	gmoSetHeadnTail(gmo, gmoHresused, osresult.getTimeValue());

	if (osresult.getSolutionNumber() == 0) return;

	OptimizationSolution* sol = osresult.optimization->solution[0];
	assert(sol != NULL);

	double* colMarg = CoinCopyOfArray((double*) NULL, gmoN(gmo), gmoValNA(gmo));
	double* colLev = CoinCopyOfArray((double*) NULL, gmoN(gmo), gmoValNA(gmo));
	double* rowLev = CoinCopyOfArray((double*) NULL, gmoM(gmo), gmoValNA(gmo));
	double* rowMarg = CoinCopyOfArray((double*) NULL, gmoM(gmo), gmoValNA(gmo));

	// TODO use get-functions

	// TODO constraint values
//	if (sol->constraints && sol->constraints->values) // set row levels, if available
//		for (std::vector<ConValue*>::iterator it(sol->constraints->values->con.begin());
//				it!=sol->constraints->values->con.end(); ++it)
//			rowLev[(*it)->idx]=(*it)->value;

	// set row dual values, if available
	if (sol->constraints != NULL && sol->constraints->dualValues != NULL) for (int i = 0;
			i < sol->constraints->dualValues->numberOfCon; ++i)
		rowMarg[sol->constraints->dualValues->con[i]->idx] = sol->constraints->dualValues->con[i]->value;

	// set var values, if available
	if (sol->variables != NULL && sol->variables->values != NULL) for (int i = 0;
			i < sol->variables->values->numberOfVar; ++i)
		colLev[sol->variables->values->var[i]->idx] = sol->variables->values->var[i]->value;

	if (sol->variables != NULL) for (int i = 0; i < sol->variables->numberOfOtherVariableResults; ++i)
		if (sol->variables->other[i]->name == "reduced costs")
		{
			for (int j = 0; j < sol->variables->other[i]->numberOfVar; ++j)
				colMarg[sol->variables->other[i]->var[j]->idx] = atof(sol->variables->other[i]->var[j]->value.c_str());
			break;
		}

	gmoSetSolution(gmo, colLev, colMarg, rowMarg, rowLev);

	if (gmoModelType(gmo) == gmoProc_cns) switch( gmoModelStat(gmo) )
	{
		case gmoModelStat_OptimalGlobal:
		case gmoModelStat_OptimalLocal:
		case gmoModelStat_Feasible:
		case gmoModelStat_Integer:
		gmoModelStatSet(gmo, gmoModelStat_Solved);
	}

	delete[] rowLev;
	delete[] rowMarg;
	delete[] colLev;
	delete[] colMarg;
}

void GAMS2OS::writeResult(ProcessInfo& info)
{
	int numPrimalSols = info.primalSolutions.size();

	gmoSetHeadnTail(gmo, gmoTmipbest, info.getDualBound());
	gmoSetHeadnTail(gmo, gmoHresused, info.getElapsedTime("Total"));

	if (!info.primalSolutions.empty())
	{
		assert(info.primalSolutions[0].point.size() == gmoN(gmo));
		gmoSetSolutionPrimal(gmo, &info.primalSolutions[0].point[0]);
	}

	gmoSolveStatSet(gmo, gmoSolveStat_Normal);
	gmoModelStatSet(gmo, gmoModelStat_NoSolutionReturned);

	switch (info.getCurrentIteration()->solutionStatus)
	{
		case E_ProblemSolutionStatus::Optimal:
			assert(!info.primalSolutions.empty());
			gmoModelStatSet(gmo, gmoModelStat_OptimalGlobal);
			break;

		case E_ProblemSolutionStatus::Feasible:
			assert(!info.primalSolutions.empty());
			gmoModelStatSet(gmo, gmoNDisc(gmo) > 0 ? gmoModelStat_Integer : gmoModelStat_Feasible);
			break;

		case E_ProblemSolutionStatus::Unbounded:
			gmoModelStatSet(gmo,
					info.primalSolutions.empty() ? gmoModelStat_UnboundedNoSolution : gmoModelStat_Unbounded);
			break;

		case E_ProblemSolutionStatus::Error:
			gmoModelStatSet(gmo,
					info.primalSolutions.empty() ?
							gmoModelStat_ErrorNoSolution :
							(gmoNDisc(gmo) > 0 ? gmoModelStat_Integer : gmoModelStat_Feasible));
			gmoSolveStatSet(gmo, gmoSolveStat_SolverErr);
			break;

		case E_ProblemSolutionStatus::Infeasible:
			assert(info.primalSolutions.empty());
			gmoModelStatSet(gmo, gmoModelStat_InfeasibleNoSolution);
			break;

		case E_ProblemSolutionStatus::IterationLimit:
			gmoModelStatSet(gmo,
					info.primalSolutions.empty() ?
							gmoModelStat_NoSolutionReturned :
							(gmoNDisc(gmo) > 0 ? gmoModelStat_Integer : gmoModelStat_Feasible));
			gmoSolveStatSet(gmo, gmoSolveStat_Iteration);
			break;

		case E_ProblemSolutionStatus::SolutionLimit:
			gmoModelStatSet(gmo,
					info.primalSolutions.empty() ?
							gmoModelStat_NoSolutionReturned :
							(gmoNDisc(gmo) > 0 ? gmoModelStat_Integer : gmoModelStat_Feasible));
			gmoSolveStatSet(gmo, gmoSolveStat_Solver);
			break;

		case E_ProblemSolutionStatus::TimeLimit:
			gmoModelStatSet(gmo,
					info.primalSolutions.empty() ?
							gmoModelStat_NoSolutionReturned :
							(gmoNDisc(gmo) > 0 ? gmoModelStat_Integer : gmoModelStat_Feasible));
			gmoSolveStatSet(gmo, gmoSolveStat_Resource);
			break;

		default:
			gmoModelStatSet(gmo,
					info.primalSolutions.empty() ?
							gmoModelStat_ErrorNoSolution :
							(gmoNDisc(gmo) > 0 ? gmoModelStat_Integer : gmoModelStat_Feasible));
			gmoSolveStatSet(gmo, gmoSolveStat_SystemErr);
			gevLogStat(gev, "Unknown solution status returned from SHOT.");
			break;
	}

	if (gmoModelType(gmo) == gmoProc_cns) switch( gmoModelStat(gmo) )
	{
		case gmoModelStat_OptimalGlobal:
		case gmoModelStat_OptimalLocal:
		case gmoModelStat_Feasible:
		case gmoModelStat_Integer:
		gmoModelStatSet(gmo, gmoModelStat_Solved);
	}
}

void GAMS2OS::clear()
{
	if (gmo == NULL) return;

	gmoUnloadSolutionLegacy (gmo);

	gmoFree(&gmo);
	gmo = NULL;

	assert(gev != NULL);
	gevFree (&gev);
	gev = NULL;

	/* remove temporary directory content (should have only files) and directory itself) */
	if (createdtmpdir)
	{
		system("rm loadgms.tmp/* && rmdir loadgms.tmp");
		createdtmpdir = false;
	}

	delete osinstance;
	osinstance = NULL;

	delete osoptions;
	osoptions = NULL;
}
