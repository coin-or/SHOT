/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Stefan Vigerske, GAMS Development Corp.
   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "IModelingSystem.h"

#include "gmomcc.h"
#include "gevmcc.h"
#include "palmcc.h"

#include "spdlog/spdlog.h"
#include "spdlog/sinks/base_sink.h"

#include <memory>
#include <string>

namespace SHOT
{

class NonlinearExpression;
using NonlinearExpressionPtr = std::shared_ptr<NonlinearExpression>;

enum class E_GAMSInputSource
{
    ProblemFile,
    GAMSModel
};

class ModelingSystemGAMS : public IModelingSystem
{
public:
    ModelingSystemGAMS(EnvironmentPtr envPtr);
    ~ModelingSystemGAMS() override;

    void setModelingObject(gmoHandle_t gmo);

    // Adds modeling system specific settings
    static void augmentSettings(SettingsPtr settings);

    // Get specific settings from modeling system
    void updateSettings(SettingsPtr settings) override;

    // Create the optimization problem by filename either directly from a gms-file or from a compiled GAMS model
    E_ProblemCreationStatus createProblem(
        ProblemPtr& problem, const std::string& filename, const E_GAMSInputSource& inputSource);

    // Create the optimization problem from the stored GAMS model instance object
    E_ProblemCreationStatus createProblem(ProblemPtr& problem);

    // Move the solution and statistics from SHOT to the modeling system
    void finalizeSolution() override;

    gmoHandle_t modelingObject;
    gevHandle_t modelingEnvironment;
    palHandle_t auditLicensing;

private:
    bool createdtmpdir;
    std::string tmpdirname;
    bool createdgmo;
    char buffer[GMS_SSSIZE];

    void createModelFromProblemFile(const std::string& filename);
    void createModelFromGAMSModel(const std::string& filename);
    void createAuditLicensing();

    void clearGAMSObjects();

    bool copyVariables(ProblemPtr destination);
    bool copyObjectiveFunction(ProblemPtr destination);
    bool copyConstraints(ProblemPtr destination);
    bool copyLinearTerms(ProblemPtr destination);
    bool copyQuadraticTerms(ProblemPtr destination);
    bool copyNonlinearExpressions(ProblemPtr destination);
    bool copySOS(ProblemPtr destination);

    static void applyOperation(std::vector<NonlinearExpressionPtr>& stack, NonlinearExpressionPtr op, int nargs);

    NonlinearExpressionPtr parseGamsInstructions(int codelen, /**< length of GAMS instructions */
        int* opcodes, /**< opcodes of GAMS instructions */
        int* fields, /**< fields of GAMS instructions */
        int constantlen, /**< length of GAMS constants pool */
        double* constants, /**< GAMS constants pool */
        const ProblemPtr& destination);
};

using ModelingSystemGAMSPtr = std::shared_ptr<ModelingSystemGAMS>;

class GamsOutputSink : public spdlog::sinks::base_sink<std::mutex>
{
private:
    gevHandle_t gev;
    GamsOutputSink() = delete;

public:
    GamsOutputSink(gevHandle_t gev_) : gev(gev_) { }

    void sink_it_(const spdlog::details::log_msg& msg) override
    {
        // log_msg is a struct containing the log entry info like level, timestamp, thread id etc.
        // msg.raw contains pre formatted log

        // If needed (very likely but not mandatory), the sink formats the message before sending it to its final
        // destination:
        spdlog::memory_buf_t formatted;
        base_sink<std::mutex>::formatter_->format(msg, formatted);

        if(msg.level <= spdlog::level::warn)
            gevLogStatPChar(gev, fmt::to_string(formatted).c_str());
        else
            gevLogPChar(gev, fmt::to_string(formatted).c_str());
    }

    void flush_() override { gevLogStatFlush(gev); }
};

} // namespace SHOT