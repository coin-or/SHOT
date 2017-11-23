#pragma once

#include "OSInstance.h"
#include "OSResult.h"
#include "OSOption.h"
#include "OSoLReader.h"

#include "gmomcc.h"
#include "gevmcc.h"
#include "GamsNLinstr.h"

class ProcessInfo;

class GAMSOSInstance: public OSInstance
{
	public:
		gmoHandle_t gmo;

		GAMSOSInstance(gmoHandle_t gmo_) :
				gmo(gmo_), OSInstance()
		{
		}

};

class GAMS2OS
{
	private:
		gmoHandle_t gmo;
		gevHandle_t gev;
		bool createdtmpdir;

		OSnLNode* parseGamsInstructions(int codelen, /**< length of GAMS instructions */
		int* opcodes, /**< opcodes of GAMS instructions */
		int* fields, /**< fields of GAMS instructions */
		int constantlen, /**< length of GAMS constants pool */
		double* constants /**< GAMS constants pool */
		);

	public:
		GAMS2OS();
		~GAMS2OS();
		void readGms(const std::string& filename);
		void readCntr(const std::string& filename);
		void writeResult(OSResult& osresult);
		void writeResult(ProcessInfo& info);
		void clear();
		void createOSObjects();

		OSInstance* osinstance;
};
