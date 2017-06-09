#pragma once
class SolutionStrategy
{
public:

	class Builder;
	//SolutionStrategy();
	//~SolutionStrategy();

private:
	int i;
	float f;
	char c;

	// Only one simple constructor - rest is handled by Builder
	SolutionStrategy();

public:
	// Product specific functionality
	void print();
	void doSomething();
	void doSomethingElse();

};

