/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "ModelingSystemAMPL.h"

#include "../Output.h"
#include "../Results.h"
#include "../Settings.h"
#include "../Timing.h"
#include "../Utilities.h"

#include "../Model/NonlinearExpressions.h"
#include "../Model/Problem.h"
#include "../Model/Simplifications.h"
#include "../Model/Variables.h"

#include "mp/nl.h"
#include "mp/problem.h"
#include "mp/nl-reader.h"
#include "mp/sol.h"

#include <cstdlib>
#include <cstring>

#ifdef HAS_STD_FILESYSTEM
#include <filesystem>
namespace fs = std;
#endif

#ifdef HAS_STD_EXPERIMENTAL_FILESYSTEM
#include <experimental/filesystem>
namespace fs = std::experimental;
#endif

#include <memory>
#include <stdexcept>
#include <string>

namespace SHOT
{

using MPProblem = mp::Problem;
using MPProblemPtr = std::shared_ptr<MPProblem>;

class AMPLProblemHandler : public mp::NullNLHandler<NonlinearExpressionPtr>
{
private:
    EnvironmentPtr env;
    ProblemPtr destination;

    NonlinearExpressions nonlinearExpressions;

    double minLBCont;
    double maxUBCont;
    double minLBInt;
    double maxUBInt;

    // SOS constraints
    // collected while handling suffixes in SuffixHandler
    // sosvars maps the SOS index (can be negative) to the indices of the variables in the SOS
    // sosweights gives for each variable its weight in the SOS it appears in (if any)
    std::map<int, std::vector<int>> sosvars;
    std::vector<int> sosweights;

    void reset() { nonlinearExpressions.clear(); }

public:
    AMPLProblemHandler(EnvironmentPtr envPtr, ProblemPtr problem) : env(envPtr), destination(problem)
    {
        this->minLBCont = env->settings->getSetting<double>("Variables.Continuous.MinimumLowerBound", "Model");
        this->maxUBCont = env->settings->getSetting<double>("Variables.Continuous.MaximumUpperBound", "Model");
        this->minLBInt = env->settings->getSetting<double>("Variables.Integer.MinimumLowerBound", "Model");
        this->maxUBInt = env->settings->getSetting<double>("Variables.Integer.MaximumUpperBound", "Model");
    }

    void OnHeader(const mp::NLHeader& h)
    {
        destination->allVariables.reserve(h.num_vars);
        destination->integerVariables.reserve(h.num_integer_vars());
        destination->realVariables.reserve(h.num_continuous_vars());

        // Creates the options header needed for the sol-file in AMPL
        if(h.num_ampl_options > 0)
        {
            std::stringstream solHeader;

            solHeader << h.num_ampl_options << '\n';

            for(int i = 0; i < h.num_ampl_options; i++)
                solHeader << h.ampl_options[i] << '\n';

            env->settings->updateSetting("AMPL.OptionsHeader", "ModelingSystem", solHeader.str());
        }

        int variableIndex = 0;

        // Nonlinear variables in both constraints and objective

        int number = h.num_nl_vars_in_both - h.num_nl_integer_vars_in_both;

        for(int i = 0; i < number; i++)
        {
            destination->add(std::make_shared<SHOT::Variable>(
                "x_" + std::to_string(variableIndex), variableIndex, E_VariableType::Real, SHOT_DBL_MIN, SHOT_DBL_MAX));
            variableIndex++;
        }

        number = h.num_nl_integer_vars_in_both;

        for(int i = 0; i < number; i++)
        {
            destination->add(std::make_shared<SHOT::Variable>("i_" + std::to_string(variableIndex), variableIndex,
                E_VariableType::Integer, SHOT_DBL_MIN, SHOT_DBL_MAX));
            variableIndex++;
        }

        // Nonlinear variables in constraints

        number = (h.num_nl_vars_in_cons - h.num_nl_vars_in_both) - h.num_nl_integer_vars_in_cons;

        for(int i = 0; i < number; i++)
        {
            destination->add(std::make_shared<SHOT::Variable>(
                "x_" + std::to_string(variableIndex), variableIndex, E_VariableType::Real, SHOT_DBL_MIN, SHOT_DBL_MAX));
            variableIndex++;
        }

        number = h.num_nl_integer_vars_in_cons;

        for(int i = 0; i < number; i++)
        {
            destination->add(std::make_shared<SHOT::Variable>("i_" + std::to_string(variableIndex), variableIndex,
                E_VariableType::Integer, SHOT_DBL_MIN, SHOT_DBL_MAX));
            variableIndex++;
        }

        // Nonlinear variables in objective

        number = (h.num_nl_vars_in_objs - h.num_nl_vars_in_cons) - h.num_nl_integer_vars_in_objs;

        for(int i = 0; i < number; i++)
        {
            destination->add(std::make_shared<SHOT::Variable>(
                "x_" + std::to_string(variableIndex), variableIndex, E_VariableType::Real, SHOT_DBL_MIN, SHOT_DBL_MAX));
            variableIndex++;
        }

        number = h.num_nl_integer_vars_in_objs;

        for(int i = 0; i < number; i++)
        {
            destination->add(std::make_shared<SHOT::Variable>("i_" + std::to_string(variableIndex), variableIndex,
                E_VariableType::Integer, SHOT_DBL_MIN, SHOT_DBL_MAX));
            variableIndex++;
        }

        // Linear variables real

        number = (h.num_vars - variableIndex) - (h.num_linear_binary_vars + h.num_linear_integer_vars);

        for(int i = 0; i < number; i++)
        {
            destination->add(std::make_shared<SHOT::Variable>(
                "x_" + std::to_string(variableIndex), variableIndex, E_VariableType::Real, SHOT_DBL_MIN, SHOT_DBL_MAX));
            variableIndex++;
        }

        // Linear variables binaries

        number = h.num_linear_binary_vars;

        for(int i = 0; i < number; i++)
        {
            destination->add(std::make_shared<SHOT::Variable>(
                "b_" + std::to_string(variableIndex), variableIndex, E_VariableType::Binary));
            variableIndex++;
        }

        number = h.num_linear_integer_vars;

        for(int i = 0; i < number; i++)
        {
            destination->add(std::make_shared<SHOT::Variable>("i_" + std::to_string(variableIndex), variableIndex,
                E_VariableType::Integer, -SHOT_INT_MAX, SHOT_INT_MAX));
            variableIndex++;
        }

        assert(variableIndex == h.num_vars);

        env->settings->updateSetting("AMPL.NumberOfOriginalConstraints", "ModelingSystem", h.num_algebraic_cons);

        destination->numericConstraints.reserve(h.num_algebraic_cons);
        destination->linearConstraints.reserve(h.num_algebraic_cons - h.num_nl_cons);
        destination->nonlinearConstraints.reserve(h.num_nl_cons);

        for(int i = 0; i < h.num_nl_cons; i++)
        {
            destination->add(
                std::make_shared<NonlinearConstraint>(i, "nlc_" + std::to_string(i), SHOT_DBL_MIN, SHOT_DBL_MAX));
        }

        for(int i = h.num_nl_cons; i < h.num_algebraic_cons; i++)
        {
            destination->add(
                std::make_shared<LinearConstraint>(i, "lc_" + std::to_string(i), SHOT_DBL_MIN, SHOT_DBL_MAX));
        }

        if(h.num_nl_objs == 1)
        {
            destination->add(std::make_shared<NonlinearObjectiveFunction>());
        }
        else if(h.num_nl_objs == 0)
        {
            destination->add(std::make_shared<LinearObjectiveFunction>());
        }
    }

    NonlinearExpressionPtr OnNumber(double value) { return std::make_shared<ExpressionConstant>(value); }

    NonlinearExpressionPtr OnVariableRef(int variableIndex)
    {
        auto variable = destination->getVariable(variableIndex);

        if(variable->lowerBound == variable->upperBound)
            return std::make_shared<ExpressionConstant>(variable->lowerBound);

        return std::make_shared<ExpressionVariable>(variable);
    }

    NonlinearExpressionPtr OnUnary(mp::expr::Kind kind, NonlinearExpressionPtr child)
    {
        switch(kind)
        {

        case mp::expr::MINUS:
            return std::make_shared<ExpressionNegate>(child);

        case mp::expr::ABS:
            return std::make_shared<ExpressionAbs>(child);

        case mp::expr::POW2:
            return std::make_shared<ExpressionSquare>(child);

        case mp::expr::SQRT:
            return std::make_shared<ExpressionSquareRoot>(child);

        case mp::expr::LOG:
            return std::make_shared<ExpressionLog>(child);

        case mp::expr::LOG10:
            return std::make_shared<ExpressionProduct>(
                std::make_shared<ExpressionConstant>(1.0 / log(10.0)), std::make_shared<ExpressionLog>(child));

        case mp::expr::EXP:
            return std::make_shared<ExpressionExp>(child);

        case mp::expr::SIN:
            return std::make_shared<ExpressionSin>(child);

        case mp::expr::COS:
            return std::make_shared<ExpressionCos>(child);

        case mp::expr::TAN:
            return std::make_shared<ExpressionTan>(child);

        case mp::expr::ASIN:
            return std::make_shared<ExpressionArcSin>(child);

        case mp::expr::ACOS:
            return std::make_shared<ExpressionArcCos>(child);

        case mp::expr::ATAN:
            return std::make_shared<ExpressionArcTan>(child);

        default:
            throw OperationNotImplementedException(fmt::format("Error: Unsupported AMPL function {}", kind));
            break;
        }

        return nullptr;
    }

    NonlinearExpressionPtr OnBinary(
        mp::expr::Kind kind, NonlinearExpressionPtr firstChild, NonlinearExpressionPtr secondChild)
    {
        switch(kind)
        {
        case mp::expr::ADD:
            return std::make_shared<ExpressionSum>(firstChild, secondChild);

        case mp::expr::SUB:
            return std::make_shared<ExpressionSum>(firstChild, std::make_shared<ExpressionNegate>(secondChild));

        case mp::expr::MUL:
            return std::make_shared<ExpressionProduct>(firstChild, secondChild);

        case mp::expr::DIV:
            return std::make_shared<ExpressionDivide>(firstChild, secondChild);

        case mp::expr::POW:
            return std::make_shared<ExpressionPower>(firstChild, secondChild);

        case mp::expr::POW_CONST_BASE:
            return std::make_shared<ExpressionPower>(firstChild, secondChild);

        case mp::expr::POW_CONST_EXP:
            return std::make_shared<ExpressionPower>(firstChild, secondChild);

        default:
            throw OperationNotImplementedException(fmt::format("Error: Unsupported AMPL function {}", kind));
            break;
        }

        return nullptr;
    }

    // Used for creating a list of terms in a sum
    struct NumericArgHandler
    {
        NonlinearExpressions terms;

        void AddArg(NonlinearExpressionPtr term) { terms.add(term); }
    };

    NumericArgHandler BeginSum(int) { return NumericArgHandler(); }

    NonlinearExpressionPtr EndSum(NumericArgHandler handler) { return std::make_shared<ExpressionSum>(handler.terms); }

    void OnObj([[maybe_unused]] int objectiveIndex, mp::obj::Type type, NonlinearExpressionPtr nonlinearExpression)
    {
        if(type == mp::obj::Type::MAX)
            destination->objectiveFunction->direction = E_ObjectiveFunctionDirection::Maximize;
        else
            destination->objectiveFunction->direction = E_ObjectiveFunctionDirection::Minimize;

        if(nonlinearExpression)
        {
            if(nonlinearExpression->getType() == E_NonlinearExpressionTypes::Constant)
            {
                destination->objectiveFunction->constant += nonlinearExpression->getBounds().l();
            }
            else
            {
                std::dynamic_pointer_cast<NonlinearObjectiveFunction>(destination->objectiveFunction)
                    ->add(nonlinearExpression);
            }
        }

        reset();
    }

    void OnAlgebraicCon(int constraintIndex, NonlinearExpressionPtr nonlinearExpression)
    {
        if(nonlinearExpression)
        {
            std::dynamic_pointer_cast<NonlinearConstraint>(destination->numericConstraints[constraintIndex])
                ->add(nonlinearExpression);
        }

        reset();
    }

    void OnVarBounds(int variableIndex, double variableLB, double variableUB)
    {
        switch(destination->allVariables[variableIndex]->properties.type)
        {
        case(E_VariableType::Real):
            if(variableLB < minLBCont)
                variableLB = minLBCont;

            if(variableUB > maxUBCont)
                variableUB = maxUBCont;

            break;

        case(E_VariableType::Binary):

            if(variableLB < 0.0)
                variableLB = 0.0;

            if(variableUB > 1.0)
                variableUB = 1.0;

            break;

        case(E_VariableType::Integer):

            if(variableLB < minLBInt)
                variableLB = minLBInt;

            if(variableUB > maxUBInt)
                variableUB = maxUBInt;

            break;

        default:
            break;
        }

        destination->setVariableBounds(variableIndex, variableLB, variableUB);
    }

    void OnConBounds(int index, double lb, double ub)
    {
        if(lb == -SHOT_DBL_INF)
            lb = SHOT_DBL_MIN;

        if(ub == SHOT_DBL_INF)
            ub = SHOT_DBL_MAX;

        destination->numericConstraints[index]->valueLHS = lb;
        destination->numericConstraints[index]->valueRHS = ub;
    }

    /// handling of suffices for variable and constraint flags and SOS constraints
    ///
    /// regarding SOS in AMPL, see https://ampl.com/faqs/how-can-i-use-the-solvers-special-ordered-sets-feature/
    /// we pass the .ref suffix as weight to the SOS constraint
    /// for a SOS2, the weights determine the order of variables in the set
    template <typename T> class SuffixHandler
    {
    private:
        AMPLProblemHandler& amplph;

        // type of suffix that is handled, or SUFFIX_IGNORE if unsupported suffix
        enum
        {
            SUFFIX_IGNORE,
            SUFFIX_VARSOSNO,
            SUFFIX_VARREF,
        } suffix;

    public:
        /// constructor
        SuffixHandler(AMPLProblemHandler& amplph_, ///< problem handler
            fmtold::StringRef name, ///< name of suffix
            mp::suf::Kind kind ///< whether suffix applies to var, cons, etc
            )
            : amplph(amplph_), suffix(SUFFIX_IGNORE)
        {
            switch(kind)
            {
            case mp::suf::Kind::CON:
            {
                amplph.env->output->outputInfo(fmtold::format("Unknown constraint suffix {}. Ignoring.\n", name));
                break;
            }

            case mp::suf::Kind::VAR:
            {
                if(strncmp(name.data(), "sosno", name.size()) == 0)
                {
                    // SOS membership
                    suffix = SUFFIX_VARSOSNO;
                }
                else if(strncmp(name.data(), "ref", name.size()) == 0)
                {
                    // SOS weights
                    suffix = SUFFIX_VARREF;
                    amplph.sosweights.resize(amplph.destination->allVariables.size(), 0);
                }
                else
                {
                    amplph.env->output->outputInfo(fmtold::format("Unknown variable suffix {}. Ignoring.\n", name));
                }
                break;
            }

            case mp::suf::Kind::OBJ:
            {
                amplph.env->output->outputInfo(fmtold::format("Unknown objective suffix {}. Ignoring.\n", name));
                break;
            }

            case mp::suf::Kind::PROBLEM:
            {
                amplph.env->output->outputInfo(fmtold::format("Unknown problem suffix {}. Ignoring.\n", name));
                break;
            }
            }
        }

        void SetValue(int index, ///< index of variable, constraint, etc
            T value ///< value of suffix
        )
        {
            assert(index >= 0);
            switch(suffix)
            {
            case SUFFIX_IGNORE:
                return;

            case SUFFIX_VARSOSNO:
                // remember that variable index belongs to SOS identified by value
                amplph.sosvars[(int)value].push_back(index);
                break;

            case SUFFIX_VARREF:
                // remember that variable index has weight value
                amplph.sosweights[index] = (int)value;
                break;
            }
        }
    };

    typedef SuffixHandler<int> IntSuffixHandler;
    /// receive notification of an integer suffix
    IntSuffixHandler OnIntSuffix(fmtold::StringRef name, ///< suffix name, not null-terminated
        mp::suf::Kind kind, ///< suffix kind
        int /*num_values*/ ///< number of values to expect
    )
    {
        return IntSuffixHandler(*this, name, kind);
    }

    typedef SuffixHandler<double> DblSuffixHandler;
    /// receive notification of a double suffix
    DblSuffixHandler OnDblSuffix(fmtold::StringRef name, ///< suffix name, not null-terminated
        mp::suf::Kind kind, ///< suffix kind
        int /*num_values*/ ///< number of values to expect
    )
    {
        return DblSuffixHandler(*this, name, kind);
    }

    class LinearPartHandler
    {
    private:
        EnvironmentPtr env;
        ProblemPtr destination;

        int constraintIndex;
        bool inObjectiveFunction = false;

    public:
        explicit LinearPartHandler(EnvironmentPtr envPtr, ProblemPtr problem, int constraintIndex)
            : env(envPtr), destination(problem), constraintIndex(constraintIndex)
        {
        }

        explicit LinearPartHandler(EnvironmentPtr envPtr, ProblemPtr problem) : env(envPtr), destination(problem)
        {
            constraintIndex = -1;
            inObjectiveFunction = true;
        }

        void AddTerm(int variableIndex, double coefficient)
        {
            if(coefficient == 0.0)
                return;

            auto variable = destination->getVariable(variableIndex);

            if(variable->lowerBound == variable->upperBound)
            {
                if(inObjectiveFunction)
                    std::dynamic_pointer_cast<LinearObjectiveFunction>(destination->objectiveFunction)->constant
                        += coefficient * variable->lowerBound;
                else
                    std::dynamic_pointer_cast<LinearConstraint>(destination->numericConstraints[constraintIndex])
                        ->constant
                        += coefficient * variable->lowerBound;
            }
            else
            {
                if(inObjectiveFunction)
                    std::dynamic_pointer_cast<LinearObjectiveFunction>(destination->objectiveFunction)
                        ->add(std::make_shared<LinearTerm>(coefficient, variable));
                else
                    std::dynamic_pointer_cast<LinearConstraint>(destination->numericConstraints[constraintIndex])
                        ->add(std::make_shared<LinearTerm>(coefficient, variable));
            }
        }
    };

    typedef LinearPartHandler LinearObjHandler;

    LinearPartHandler OnLinearObjExpr([[maybe_unused]] int objectiveIndex, [[maybe_unused]] int numLinearTerms)
    {
        return LinearObjHandler(env, destination);
    }

    typedef LinearPartHandler LinearConHandler;

    LinearConHandler OnLinearConExpr(int constraintIndex, [[maybe_unused]] int numLinearTerms)
    {
        return LinearConHandler(env, destination, constraintIndex);
    }

    /// receive notification about the end of the input
    void EndInput()
    {
        for(auto sosit(sosvars.begin()); sosit != sosvars.end(); ++sosit)
        {
            assert(sosit->first != 0);
            assert(!sosit->second.empty());

            // a negative SOS identifier means SOS2
            bool issos2 = sosit->first < 0;

            if(issos2 && sosweights.empty())
            {
                // if no .ref suffix was given for a SOS2 constraint, then we consider this as an error
                // since the weights determine the order
                // for a SOS1, the weights only specify branching preference, so can treat them as optional
                OnUnhandled("SOS2 requires variable .ref suffix");
            }

            // add SOS constraints
            Variables setvars; // variables in one SOS
            setvars.resize(sosit->second.size());

            VectorDouble setweights; // weights for one SOS

            if(!sosweights.empty())
                setweights.resize(sosit->second.size());

            for(size_t i = 0; i < sosit->second.size(); ++i)
            {
                int varidx = sosit->second[i];

                if(issos2 && sosweights[varidx] == 0)
                    // 0 is the default if no ref was given for a variable; we don't allow this for SOS2
                    OnUnhandled("Missing .ref value for SOS2 variable");

                setvars[i] = destination->getVariable(varidx);

                if(!sosweights.empty())
                    setweights[i] = (double)sosweights[varidx];
            }

            destination->add(
                std::make_shared<SpecialOrderedSet>((issos2) ? E_SOSType::Two : E_SOSType::One, setvars, setweights));
        }
    }
};

ModelingSystemAMPL::ModelingSystemAMPL(EnvironmentPtr envPtr) : IModelingSystem(envPtr) { }

ModelingSystemAMPL::~ModelingSystemAMPL() = default;

void ModelingSystemAMPL::augmentSettings(SettingsPtr settings)
{
    settings->createSetting("AMPL.OptionsHeader", "ModelingSystem", std::string("0\n"),
        "The AMPL options header for the solution file", true);
    settings->createSetting("AMPL.NumberOfOriginalConstraints", "ModelingSystem", 0,
        "The number of constraints in the original problem submitted to SHOT", 0, SHOT_INT_MAX, true);
}

void ModelingSystemAMPL::updateSettings([[maybe_unused]] SettingsPtr settings) { }

E_ProblemCreationStatus ModelingSystemAMPL::createProblem(ProblemPtr& problem, const std::string& filename)
{
    if(!fs::filesystem::exists(fs::filesystem::path(filename)))
    {
        env->output->outputError("Problem file \"" + filename + "\" does not exist.");

        env->timing->stopTimer("ProblemInitialization");
        return (E_ProblemCreationStatus::FileDoesNotExist);
    }

    env->timing->startTimer("ProblemInitialization");

    fs::filesystem::path problemFile(filename);
    fs::filesystem::path problemPath = problemFile.parent_path();

    try
    {
        AMPLProblemHandler handler(env, problem);
        mp::ReadNLFile(filename, handler);
    }
    catch(const std::exception& e)
    {
        env->output->outputError(fmt::format("Error when reading AMPL model from \"{}\": {}", filename, e.what()));

        env->timing->stopTimer("ProblemInitialization");
        return (E_ProblemCreationStatus::Error);
    }

    auto colFile = fs::filesystem::path(filename).replace_extension(".col");

    if(fs::filesystem::exists(colFile))
    {
        auto variableNames = Utilities::getLinesInFile(colFile.string());

        if(variableNames.size() != problem->allVariables.size())
        {
            env->output->outputError(fmt::format(
                "Error when reading AMPL model (variable names in col-file \"{}\" does not match).", colFile.string()));
            return (E_ProblemCreationStatus::Error);
        }

        for(size_t i = 0; i < variableNames.size(); i++)
            problem->allVariables[i]->name = variableNames[i];
    }

    auto rowFile = fs::filesystem::path(filename).replace_extension(".row");

    if(fs::filesystem::exists(rowFile))
    {
        auto constraintNames = Utilities::getLinesInFile(rowFile.string());

        if(constraintNames.size() != problem->numericConstraints.size() + 1) // Last one is objective
        {
            env->output->outputError(
                fmt::format("Error when reading AMPL model (constraint names in row-file \"{}\" does not match).",
                    colFile.string()));
            return (E_ProblemCreationStatus::Error);
        }

        for(size_t i = 0; i < problem->numericConstraints.size(); i++)
            problem->numericConstraints[i]->name = constraintNames[i];
    }

    problem->updateProperties();

    bool extractMonomialTerms = env->settings->getSetting<bool>("Reformulation.Monomials.Extract", "Model");
    bool extractSignomialTerms = env->settings->getSetting<bool>("Reformulation.Signomials.Extract", "Model");
    bool extractQuadraticTerms = (env->settings->getSetting<int>("Reformulation.Quadratics.ExtractStrategy", "Model")
        >= static_cast<int>(ES_QuadraticTermsExtractStrategy::ExtractTermsToSame));

    simplifyNonlinearExpressions(problem, extractMonomialTerms, extractSignomialTerms, extractQuadraticTerms);

    problem->finalize();

    env->timing->stopTimer("ProblemInitialization");
    return (E_ProblemCreationStatus::NormalCompletion);
}

void ModelingSystemAMPL::finalizeSolution() { }

} // Namespace SHOT
