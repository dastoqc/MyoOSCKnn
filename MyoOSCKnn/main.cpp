// Copyright (C) 2013-2014 Thalmic Labs Inc.
// Distributed under the Myo SDK license agreement. See LICENSE.txt for details.
#define _USE_MATH_DEFINES
#include <array>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <algorithm>
#include <sstream>
#include <string>
// The only file that needs to be included to use the Myo C++ SDK is myo.hpp.
#include <myo/myo.hpp>

#include "classification.h"
#include <Windows.h>


#include "osc/OscOutboundPacketStream.h"
#include "ip/UdpSocket.h"
#define ADDRESS "127.0.0.1"
#define PORT 12000
#define OUTPUT_BUFFER_SIZE 1024
#define EMG_BUFFER_SIZE 10


//Key code definition
#define KEY_0 48
#define KEY_1 49
#define KEY_2 50
#define KEY_3 51
#define KEY_4 52
#define KEY_5 53
#define KEY_6 54
#define KEY_7 55
#define KEY_8 56
#define KEY_9 57

// Classes that inherit from myo::DeviceListener can be used to receive events from Myo devices. DeviceListener
// provides several virtual functions for handling different kinds of events. If you do not override an event, the
// default behavior is to do nothing.
class DataCollector : public myo::DeviceListener {
public:
	DataCollector()
		: onArm(false), isUnlocked(false), roll_w(0), pitch_w(0), yaw_w(0), currentPose()
	{
		pythonBridgeclassification = classification();
		for (int i = 0; i < 8; i++)
			emgSamples[i] = 0;
		for (int i = 0; i < EMG_BUFFER_SIZE; i++)
			emgSum[i] = 0;
		tim[0] = 0;
		for (int i = 1; i < EMG_BUFFER_SIZE; i++)
			tim[i] = tim[i - 1] + 1.0 / 200.0;
	}
	
	// onUnpair() is called whenever the Myo is disconnected from Myo Connect by the user.
	void onUnpair(myo::Myo* myo, uint64_t timestamp)
	{
		// We've lost a Myo.
		// Let's clean up some leftover state.
		roll_w = 0;
		pitch_w = 0;
		yaw_w = 0;
		onArm = false;
		isUnlocked = false;

		for (int i = 0; i < 8;i++)
			emgSamples[i] = 0;
		for (int i = 0; i < EMG_BUFFER_SIZE; i++)
			emgSum[i] = 0;
		tim[0] = 0;
		for (int i = 1; i < EMG_BUFFER_SIZE; i++)
			tim[i] = tim[i - 1] + 1.0 / 200.0;


	}

	// onEmgData() is called whenever a paired Myo has provided new EMG data, and EMG streaming is enabled.
	void onEmgData(myo::Myo* myo, uint64_t timestamp, const int8_t* emg)
	{
		long sum = 0;
		for (int i = 0; i < 8; i++) {
			emgSamples[i] = static_cast<int>(emg[i]);
			sum += abs(emgSamples[i]);
		}
		for (unsigned int i = 1; i < EMG_BUFFER_SIZE; i++)
			emgSum[i] = emgSum[i + 1];
		emgSum[EMG_BUFFER_SIZE - 1] = sum;
	}

	// onOrientationData() is called whenever the Myo device provides its current orientation, which is represented
	// as a unit quaternion.
	void onOrientationData(myo::Myo* myo, uint64_t timestamp, const myo::Quaternion<float>& quat)
	{
		using std::atan2;
		using std::asin;
		using std::sqrt;
		using std::max;
		using std::min;
		float roll, pitch, yaw;
		// Calculate Euler angles (roll, pitch, and yaw) from the unit quaternion.
		roll = atan2(2.0f * (quat.w() * quat.x() + quat.y() * quat.z()),
			1.0f - 2.0f * (quat.x() * quat.x() + quat.y() * quat.y()));
		pitch = asin(max(-1.0f, min(1.0f, 2.0f * (quat.w() * quat.y() - quat.z() * quat.x()))));
		yaw = atan2(2.0f * (quat.w() * quat.z() + quat.x() * quat.y()),
			1.0f - 2.0f * (quat.y() * quat.y() + quat.z() * quat.z()));
		// Convert the floating point angles in radians to a scale from 0 to 18.
		roll_w = static_cast<int>((roll + (float)M_PI) / (M_PI * 2.0f) * 18);
		pitch_w = static_cast<int>((pitch + (float)M_PI / 2.0f) / M_PI * 18);
		yaw_w = static_cast<int>((yaw + (float)M_PI) / (M_PI * 2.0f) * 18);
	}
	// onPose() is called whenever the Myo detects that the person wearing it has changed their pose, for example,
	// making a fist, or not making a fist anymore.
	void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose)
	{
		currentPose = pose;
		if (pose != myo::Pose::unknown && pose != myo::Pose::rest) {
			// Tell the Myo to stay unlocked until told otherwise. We do that here so you can hold the poses without the
			// Myo becoming locked.
			myo->unlock(myo::Myo::unlockHold);
			// Notify the Myo that the pose has resulted in an action, in this case changing
			// the text on the screen. The Myo will vibrate.
			myo->notifyUserAction();
		}
		else {
			// Tell the Myo to stay unlocked only for a short period. This allows the Myo to stay unlocked while poses
			// are being performed, but lock after inactivity.
			myo->unlock(myo::Myo::unlockTimed);
		}
	}
	// onArmSync() is called whenever Myo has recognized a Sync Gesture after someone has put it on their
	// arm. This lets Myo know which arm it's on and which way it's facing.
	void onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection, float rotation,
		myo::WarmupState warmupState)
	{
		onArm = true;
		whichArm = arm;
	}
	// onArmUnsync() is called whenever Myo has detected that it was moved from a stable position on a person's arm after
	// it recognized the arm. Typically this happens when someone takes Myo off of their arm, but it can also happen
	// when Myo is moved around on the arm.
	void onArmUnsync(myo::Myo* myo, uint64_t timestamp)
	{
		onArm = false;
	}
	// onUnlock() is called whenever Myo has become unlocked, and will start delivering pose events.
	void onUnlock(myo::Myo* myo, uint64_t timestamp)
	{
		isUnlocked = true;
	}
	// onLock() is called whenever Myo has become locked. No pose events will be sent until the Myo is unlocked again.
	void onLock(myo::Myo* myo, uint64_t timestamp)
	{
		isUnlocked = false;
	}



	void record_data(int classe_to_record)
	{
		int emg[8];
		int imu[3];
		int i;
		for (i = 0; i < 8; i++) {
			emg[i] = static_cast<int>(emgSamples[i]);
		}
		imu[0] = roll_w;
		imu[1] = pitch_w;
		imu[2] = yaw_w;

		pythonBridgeclassification.record_data_in_python(emg, imu,classe_to_record);

	}
	void closePython()
	{
		pythonBridgeclassification.finalize_python();
	}

	void train()
	{
		pythonBridgeclassification.train_classifier();
	}

	int classify()
	{
		int emg[8];
		int imu[3];
		int i;
		for (i = 0; i < 8; i++) {
			emg[i] = static_cast<int>(emgSamples[i]);
		}
		imu[0] = roll_w;
		imu[1] = pitch_w;
		imu[2] = yaw_w;
		int class_found = pythonBridgeclassification.classify_rf(emg, imu);
		return class_found;
	}
	// There are other virtual functions in DeviceListener that we could override here, like onAccelerometerData().
	// For this example, the functions overridden above are sufficient.
	// We define this function to print the current values that were updated by the on...() functions above.
	void print()
	{
		// Clear the current line
		std::cout << '\r';
		// Print out the orientation. Orientation data is always available, even if no arm is currently recognized.
		/*std::cout << '[' << std::string(roll_w, '*') << std::string(18 - roll_w, ' ') << ']'
		<< '[' << std::string(pitch_w, '*') << std::string(18 - pitch_w, ' ') << ']'
		<< '[' << std::string(yaw_w, '*') << std::string(18 - yaw_w, ' ') << ']';*/
		std::cout << roll_w << ", " << pitch_w << ", " << yaw_w << " ";
		// Print out the EMG data.
		for (size_t i = 0; i < 8; i++) {
			std::cout << '[' << static_cast<int>(emgSamples[i]) << ']';
		}

		/*if (onArm) {
		// Print out the lock state, the currently recognized pose, and which arm Myo is being worn on.
		// Pose::toString() provides the human-readable name of a pose. We can also output a Pose directly to an
		// output stream (e.g. std::cout << currentPose;). In this case we want to get the pose name's length so
		// that we can fill the rest of the field with spaces below, so we obtain it as a string using toString().
		std::string poseString = currentPose.toString();
		std::cout << '[' << (isUnlocked ? "unlocked" : "locked  ") << ']'
		<< '[' << (whichArm == myo::armLeft ? "L" : "R") << ']'
		<< '[' << poseString << std::string(14 - poseString.size(), ' ') << ']';
		}
		else {
		// Print out a placeholder for the arm and pose when Myo doesn't currently know which arm it's on.
		std::cout << '[' << std::string(8, ' ') << ']' << "[?]" << '[' << std::string(14, ' ') << ']';
		}*/
		std::cout << std::flush;
	}
	// These values are set by onArmSync() and onArmUnsync() above.
	bool onArm;
	myo::Arm whichArm;
	// This is set by onUnlocked() and onLocked() above.
	bool isUnlocked;
	// These values are set by onOrientationData() and onPose() above.
	int roll_w, pitch_w, yaw_w;
	myo::Pose currentPose;
	int emgSamples[8];
	double emgSum[EMG_BUFFER_SIZE];
	double tim[EMG_BUFFER_SIZE];
	classification pythonBridgeclassification;
};
DataCollector collector;

double remap(double value, double istart, double istop, double ostart, double ostop) {
	return ostart + (ostop - ostart) * ((value - istart) / (istop - istart));
}

/*------------------
//Simple example of sending an OSC message using oscpack.
-------------------*/

int sendOSC(int classdetect) {
	UdpTransmitSocket transmitSocket(IpEndpointName(ADDRESS, PORT));

	char buffer[OUTPUT_BUFFER_SIZE];
	osc::OutboundPacketStream p(buffer, OUTPUT_BUFFER_SIZE);

	p << osc::BeginBundleImmediate
		<< osc::BeginMessage("/dxl/0/G");
	switch (classdetect) {
		case 1:
			p << (float)-1 << osc::EndMessage << osc::EndBundle;
			transmitSocket.Send(p.Data(), p.Size());
			break;
		case 2:
			p << (float)1 << osc::EndMessage << osc::EndBundle;
			transmitSocket.Send(p.Data(), p.Size());
			break;
		case 3:
			p << (float)0 << osc::EndMessage << osc::EndBundle;
			transmitSocket.Send(p.Data(), p.Size());
			break;
	}
	//float mot = remap(force, 5, 30, -0.4, 0.25);
	//0.25 -> -0.4
	//float vit = remap(collector.yaw_w, 0, 18, -0.4, 0.25);

	/*p << osc::BeginBundleImmediate
		<< osc::BeginMessage("/dxl/0/S") << (float)1 << osc::EndMessage << osc::EndBundle;*/

	
	// If a standard exception occurred, we print out its message and exit.

	return 1;
}

int main(int argc, char** argv)
{
	// We catch any exceptions that might occur below -- see the catch statement for more details.
	try {
		// First, we create a Hub with our application identifier. Be sure not to use the com.example namespace when
		// publishing your application. The Hub provides access to one or more Myos.
		myo::Hub hub("com.example.hello-myo");
		std::cout << "Attempting to find a Myo..." << std::endl;
		// Next, we attempt to find a Myo to use. If a Myo is already paired in Myo Connect, this will return that Myo
		// immediately.
		// waitForMyo() takes a timeout value in milliseconds. In this case we will try to find a Myo for 10 seconds, and
		// if that fails, the function will return a null pointer.
		myo::Myo* myo = hub.waitForMyo(10000);
		// If waitForMyo() returned a null pointer, we failed to find a Myo, so exit with an error message.
		if (!myo) {
			throw std::runtime_error("Unable to find a Myo!");
		}
		// We've found a Myo.
		std::cout << "Connected to a Myo armband!" << std::endl << std::endl;
		// Next we enable EMG streaming on the found Myo.
		myo->setStreamEmg(myo::Myo::streamEmgEnabled);
		// Next we construct an instance of our DeviceListener, so that we can register it with the Hub.
		//DataCollector collector;
		// Hub::addListener() takes the address of any object whose class inherits from DeviceListener, and will cause
		// Hub::run() to send events to all registered device listeners.
		hub.addListener(&collector);
		// Finally we enter our main loop.
		bool recorded = false;
		while (1) {
			int class_found = -1;
			// In each iteration of our main loop, we run the Myo event loop for a set number of milliseconds.
			// In this case, we wish to update our display 20 times a second, so we run for 1000/20 milliseconds.
			hub.run(1000 / 50);
			// After processing events, we call the print() member function we defined above to print out the values we've
			// obtained from any events that have occurred.
			//collector.print();
			
			//Record data if key pressed
			int classe_to_record = -1;
			if (GetAsyncKeyState(KEY_0))
			{
				classe_to_record = 0;
			}
			else if (GetAsyncKeyState(KEY_1))
			{
				classe_to_record = 1;
			}
			else if (GetAsyncKeyState(KEY_2))
			{
				classe_to_record = 2;
			}
			else if (GetAsyncKeyState(KEY_3))
			{
				classe_to_record = 3;
			}
			else if (GetAsyncKeyState(KEY_4))
			{
				classe_to_record = 4;
			}
			else if (GetAsyncKeyState(KEY_5))
			{
				classe_to_record = 5;
			}
			else if (GetAsyncKeyState(KEY_6))
			{
				classe_to_record = 6;
			}
			else if (GetAsyncKeyState(KEY_7))
			{
				classe_to_record = 7;
			}
			else if (GetAsyncKeyState(KEY_8))
			{
				classe_to_record = 8;
			}
			else if (GetAsyncKeyState(KEY_9))
			{
				classe_to_record = 9;
			}
			//We record new data
			if (classe_to_record != -1)
			{
				recorded = true;
				collector.record_data(classe_to_record);
			}
			else if(recorded == true)
			{
				collector.train();
				recorded = false;
			}
			//Classify
			class_found = collector.classify();
			std::cout << class_found;
			sendOSC(class_found);
		}
		collector.closePython();
		// If a standard exception occurred, we print out its message and exit.
	}
	catch (const std::exception& e) {
		std::cerr << "Error: " << e.what() << std::endl;
		std::cerr << "Press enter to continue.";
		std::cin.ignore();
		return 1;
	}
}
