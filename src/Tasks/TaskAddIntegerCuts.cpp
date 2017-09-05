#include "TaskAddIntegerCuts.h"

TaskAddIntegerCuts::TaskAddIntegerCuts(IMILPSolver *MILPSolver)
{
	//processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();
	this->MILPSolver = MILPSolver;
}

TaskAddIntegerCuts::~TaskAddIntegerCuts()
{
	// TODO Auto-generated destructor stub
}

void TaskAddIntegerCuts::run()
{
	auto currIter = ProcessInfo::getInstance().getCurrentIteration(); // The unsolved new iteration

	if (ProcessInfo::getInstance().integerCutWaitingList.size() == 0) return;

	if (!currIter->isMILP() || !settings->getBoolSetting("DelayedConstraints", "MILP")
			|| !currIter->MILPSolutionLimitUpdated)
	{

		for (int j = 0; j < ProcessInfo::getInstance().integerCutWaitingList.size(); j++)
		{
			auto tmpBinaryCombination = ProcessInfo::getInstance().integerCutWaitingList.at(j);
			int numOnes = tmpBinaryCombination.size();

			std::vector < IndexValuePair > elements;

			for (int i = 0; i < numOnes; i++)
			{
				IndexValuePair pair;
				pair.idx = tmpBinaryCombination.at(i);
				pair.value = 1.0;

				elements.push_back(pair);
			}

			this->MILPSolver->addLinearConstraint(elements, -(numOnes - 1.0));
			ProcessInfo::getInstance().numIntegerCutsAdded++;
		}

		ProcessInfo::getInstance().outputInfo(
				"     Added " + to_string(ProcessInfo::getInstance().integerCutWaitingList.size())
						+ " integer cut(s).                                        ");

		ProcessInfo::getInstance().integerCutWaitingList.clear();
	}
}

std::string TaskAddIntegerCuts::getType()
{
	std::string type = typeid(this).name();
	return (type);

}
