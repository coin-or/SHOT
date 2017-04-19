#include <TaskAddIntegerCuts.h>

TaskAddIntegerCuts::TaskAddIntegerCuts()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();
}

TaskAddIntegerCuts::~TaskAddIntegerCuts()
{
	// TODO Auto-generated destructor stub
}

void TaskAddIntegerCuts::run()
{
	auto currIter = processInfo->getCurrentIteration(); // The unsolved new iteration

	if (processInfo->integerCutWaitingList.size() == 0) return;

	if (!currIter->isMILP() || !settings->getBoolSetting("DelayedConstraints", "MILP")
			|| !currIter->MILPSolutionLimitUpdated)
	{

		for (int j = 0; j < processInfo->integerCutWaitingList.size(); j++)
		{
			auto tmpBinaryCombination = processInfo->integerCutWaitingList.at(j);
			int numOnes = tmpBinaryCombination.size();

			std::vector < IndexValuePair > elements;

			for (int i = 0; i < numOnes; i++)
			{
				IndexValuePair pair;
				pair.idx = tmpBinaryCombination.at(i);
				pair.value = 1.0;

				elements.push_back(pair);
			}

			processInfo->MILPSolver->addLinearConstraint(elements, -(numOnes - 1.0));
			processInfo->numIntegerCutsAdded++;
		}

		processInfo->outputInfo(
				"     Added " + to_string(processInfo->integerCutWaitingList.size())
						+ " integer cut(s).                                        ");

		processInfo->integerCutWaitingList.clear();
	}
}

std::string TaskAddIntegerCuts::getType()
{
	std::string type = typeid(this).name();
	return (type);

}
