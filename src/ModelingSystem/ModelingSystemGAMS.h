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

#include <cstdio>
#include <cstdlib>
#include <sys/stat.h> // for mkdir

#include "gmomcc.h"
#include "gevmcc.h"
#include "GamsNLinstr.h"

namespace SHOT
{

enum class E_GAMSInputSource
{
    ProblemFile,
    GAMSModel
};

class ModelingSystemGAMS : public IModelingSystem
{
public:
    ModelingSystemGAMS(EnvironmentPtr envPtr);
    virtual ~ModelingSystemGAMS();

    // Adds modeling system specific settings
    virtual void augmentSettings(SettingsPtr settings);

    // Get specific settings from modeling system
    virtual void updateSettings(SettingsPtr settings);

    // Create the optimization problem by filename either directly from a gms-file or from a compiled GAMS model
    E_ProblemCreationStatus createProblem(
        ProblemPtr& problem, const std::string& filename, const E_GAMSInputSource& inputSource);

    // Create the optimization problem by filename from a GAMS model instance object
    E_ProblemCreationStatus createProblem(ProblemPtr& problem, gmoHandle_t gmo);

    // Move the solution and statistics from SHOT to the modeling system
    virtual void finalizeSolution();

    gmoHandle_t modelingObject;
    gevHandle_t modelingEnvironment;

private:
    bool createdtmpdir;
    bool createdgmo;
    char buffer[GMS_SSSIZE];

    void createModelFromProblemFile(const std::string& filename);
    void createModelFromGAMSModel(const std::string& filename);

    void clearGAMSObjects();

    bool copyVariables(ProblemPtr destination);
    bool copyObjectiveFunction(ProblemPtr destination);
    bool copyConstraints(ProblemPtr destination);
    bool copyLinearTerms(ProblemPtr destination);
    bool copyQuadraticTerms(ProblemPtr destination);
    bool copyNonlinearExpressions(ProblemPtr destination);

    static void applyOperation(std::vector<NonlinearExpressionPtr>& stack, NonlinearExpressionPtr op, int nargs);

    NonlinearExpressionPtr parseGamsInstructions(int codelen, /**< length of GAMS instructions */
        int* opcodes, /**< opcodes of GAMS instructions */
        int* fields, /**< fields of GAMS instructions */
        int constantlen, /**< length of GAMS constants pool */
        double* constants, /**< GAMS constants pool */
        const ProblemPtr& destination);
};

typedef std::shared_ptr<ModelingSystemGAMS> ModelingSystemGAMSPtr;

class GamsOutputSink : public spdlog::sinks::base_sink<std::mutex>
{
private:
    gevHandle_t gev;
    GamsOutputSink() = delete;

public:
    GamsOutputSink(gevHandle_t gev_) : gev(gev_) {}

    void sink_it_(const spdlog::details::log_msg& msg) override
    {
        // log_msg is a struct containing the log entry info like level, timestamp, thread id etc.
        // msg.raw contains pre formatted log

        // If needed (very likely but not mandatory), the sink formats the message before sending it to its final
        // destination:
        fmt::memory_buffer formatted;
        sink::formatter_->format(msg, formatted);

        if(msg.level <= spdlog::level::warn)
            gevLogStatPChar(gev, fmt::to_string(formatted).c_str());
        else
            gevLogPChar(gev, fmt::to_string(formatted).c_str());
    }

    void flush_() override { gevLogStatFlush(gev); }
};

} // namespace SHOT