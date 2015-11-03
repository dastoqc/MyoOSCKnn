#ifdef _DEBUG
	#undef _DEBUG
	#include <Python.h>
	#define _DEBUG
#else
	#include <Python.h>
#endif
#include <iostream>
#include "classification.h"
using namespace std;



PyObject *pName, *pModule, *pDict, *pClass, *pInstance;
classification::classification()
{
	char *argv[] = { "montrealProject", "testFunction", "RFClassifier", NULL };
	//Initialize the Python Interpreter
	Py_Initialize();

	//Build the name object
	pName = PyString_FromString(argv[1]);
	//Load the module object
	pModule = PyImport_Import(pName);
	// pDict is a borrowed reference
	pDict = PyModule_GetDict(pModule);
	//Build the name of the callable class
	pClass = PyDict_GetItemString(pDict, argv[2]);

	pInstance = NULL;
	// Create an instance of the class
	if (PyCallable_Check(pClass))
	{
		pInstance = PyObject_CallObject(pClass, NULL);
	}
}
void classification::finalize_python()
{
	// Clean up
	Py_DECREF(pModule);
	Py_DECREF(pName);
	Py_Finalize();
}
void classification::record_data_in_python(int emg[], int imu[],int classe_being_recorded)
{
	PyObject_CallMethod(pInstance, "store_data", "iiiiiiiiiiii", classe_being_recorded, emg[0], emg[1], emg[2], emg[3], emg[4], emg[5], emg[6], emg[7], imu[0], imu[1], imu[2]);
}

int classification::classify_rf(int emg[], int imu[])
{
	int classification = 0;
	PyObject *pValue;
	pValue = PyObject_CallMethod(pInstance, "classifyPython", "iiiiiiiiiii", emg[0], emg[1], emg[2], emg[3], emg[4], emg[5], emg[6], emg[7], imu[0], imu[1], imu[2]);

	if (pValue != NULL)
	{
		classification = PyInt_AsLong(pValue);
		Py_DECREF(pValue);
	}
	else
	{
		PyErr_Print();
	}
	// Finish the Python Interpreter
	return classification;
}

void classification::train_classifier()
{
	PyObject_CallMethod(pInstance, "read_data", "");
}
