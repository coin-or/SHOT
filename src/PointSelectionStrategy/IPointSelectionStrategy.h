#pragma once
#include "vector"

class IPointSelectionStrategy
{
public:
	IPointSelectionStrategy();
	~IPointSelectionStrategy();

	virtual std::vector<double> selectPoint() = 0;
	virtual std::vector<std::vector<double>> selectPoints(int maxNumPts) = 0;
};
