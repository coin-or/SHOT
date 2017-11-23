#include "SolutionStrategy.h"

/*
 SolutionStrategy::SolutionStrategy()
 {
 }


 SolutionStrategy::~SolutionStrategy()
 {
 }
 */

class SolutionStrategy::Builder
{
	private:
		// variables needed for construction of object of Product class
		//int i;
		//float f;
		//char c;

	public:
		// default values for variables
		//static const int defaultI = 1;
		//static const float defaultF;
		//static const char defaultC = 'a';

		// create Builder with default values assigned
		// (in C++11 they can be simply assigned above on declaration instead)
		Builder(); //: i(defaultI), f(defaultF), c(defaultC){}

		// sets custom values for Product creation
		// returns Builder for shorthand inline usage (same way as cout <<)
		//Builder& setI(const int i){ this->i = i; return *this; }
		//Builder& setF(const float f){ this->f = f; return *this; }
		//Builder& setC(const char c){ this->c = c; return *this; }

		// prepare specific frequently desired Product
		// returns Builder for shorthand inline usage (same way as cout <<)
		/*Builder& setProductP(){
		 this->i = 42;
		 this->f = -1.0f / 12.0f;
		 this->c = '@';

		 return *this;
		 }*/

		// produce desired Product
		SolutionStrategy build()
		{
			// here optionally check variable consistency
			// and also if Product is buildable from given information

			return SolutionStrategy();
		}
};
