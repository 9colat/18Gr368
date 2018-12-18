#define WIN32_LEAN_AND_MEAN

#include "SerialPort.h"

#include <iostream>
#include <thread>
#include <chrono>
#include <ratio>
#include <ctime>
#include <string>
#include <myo/myo.hpp>
#include <deque>


#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdlib.h>
#include <stdio.h>

#pragma comment (lib, "Ws2_32.lib")
#pragma comment (lib, "Mswsock.lib")
#pragma comment (lib, "AdvApi32.lib")


#define M_PI 3.14159265359
const char* serial_port = "\\\\.\\COM8";
#define DEFAULT_NETWORK_BUFLEN 255
#define DEFAULT_NETWORK_PORT "3333"

using namespace std;
using namespace myo;


class DataCollector : public DeviceListener {
	public:
	string currentPose = "";

	deque<int8_t> emg_data;

	double roll = 0.0;
	double pitch = 0.0;
	double yaw = 0.0;
	double yaw_offset = 0.0;
	bool synced = false;
	
	unsigned char roll_int = 0;
	unsigned char  pitch_int = 0;
	unsigned char  yaw_int = 0;
	unsigned char  pose_int = 0;

	void onOrientationData(Myo *myo, uint64_t time, const Quaternion<float> &rot) {
		roll = atan2(2.0f * (rot.w() * rot.x() + rot.y() * rot.z()),
			1.0f - 2.0f * (rot.x() * rot.x() + rot.y() * rot.y()));
		pitch = asin(max(-1.0f, min(1.0f, 2.0f * (rot.w() * rot.y() - rot.z() * rot.x()))));
		yaw = atan2(2.0f * (rot.w() * rot.z() + rot.x() * rot.y()),
			1.0f - 2.0f * (rot.y() * rot.y() + rot.z() * rot.z()));

		double rad_to_deg = 180.0 / M_PI;
		roll *= rad_to_deg;
		pitch *= rad_to_deg;
		yaw *= rad_to_deg;


		roll_int =  static_cast<unsigned char>( floor((roll + 180.0) / 2.0 + 0.5) );
		pitch_int = static_cast<unsigned char>( floor((pitch + 90.0)     / 2.0 + 0.5) );
		yaw_int =   static_cast<unsigned char>( floor(fmod(yaw - yaw_offset + 540.0, 360.0) / 2.0 + 0.5) );
		//cout << yaw  << " - " << yaw_offset << " = " << yaw - yaw_offset << " +360 = " << ( yaw - yaw_offset )+480.0 << ", mod 360 = " << fmod( yaw - yaw_offset + 540.0, 360.0 ) << " floor and half: " << floor( fmod(yaw - yaw_offset + 540.0, 360.0)/2.0 +0.5 ) << endl;
	
	}

	void onEmgData(myo::Myo* myo, uint64_t timestamp, const int8_t *emg){
		for (int i = 0; i < 8; i++) {
			if (emg[i] != 0) {
				emg_data.push_front(emg[i]);
				if (emg_data.size() > 400) {
					emg_data.pop_back();
				}
			}
		}
	}

	void onPose(Myo *myo, uint64_t timestamp, Pose pose) {
		currentPose = pose.toString();
		if (!synced) {
			if (currentPose == "unknown") {
				pose_int = 0;
			}
			else if (currentPose == "rest") {
				pose_int = 1;
			}
			else if (currentPose == "fist") {
				pose_int = 2;
			}
			else if (currentPose == "fingersSpread") {
				pose_int = 3;
			}
			else if (currentPose == "waveIn") {
				pose_int = 4;
			}
			else if (currentPose == "waveOut") {
				pose_int = 5;
			}
			else if (currentPose == "doubleTap") {
				pose_int = 6;
			}
		}
	}
};


int main(){
	/*///------------------------------------------------------------------------------------------------
	Connect to the Teensy
	/*///------------------------------------------------------------------------------------------------
	cout << "Establishing connection to teensy..." << endl;
	SerialPort teensy(serial_port);
	string transmitted = "HANDSHAKE:164897";
	string received = "";
	while ( received != transmitted ) {
		cout << "Sent " << transmitted.size() << "b:     " << transmitted << endl;
		teensy.writeSerialPortGood(transmitted, '\n');
		this_thread::sleep_for(200ms);

		received = teensy.readSerialPortGood('\n');
		cout << "Received " << received.size() << "b: " << received << endl;
		this_thread::sleep_for(200ms);
	}
	cout << "Teensy connected!" << endl;

	this_thread::sleep_for(1000ms);


	/*///------------------------------------------------------------------------------------------------
	Connect to the Myo band
	/*///------------------------------------------------------------------------------------------------
	cout << "Establishing connection to the Myo band..." << endl;
	Hub hub("com.robot.controller");
	Myo *myo = hub.waitForMyo(10000);
	myo->setStreamEmg(Myo::streamEmgEnabled);
	DataCollector listener;
	hub.addListener(&listener);

	if (!myo) {
		cout << "Myo connection failed" << endl;
		return 1;
	} else {
		cout << "Myo connected!" << endl;
		
	}

	chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
	hub.setLockingPolicy(hub.lockingPolicyNone);
	while ( listener.currentPose != "waveOut" ) {
		using namespace std::chrono;

		hub.runOnce(100);
		listener.yaw_offset = listener.yaw;
		if ( duration_cast< milliseconds >(steady_clock::now() - t1).count() > 1000 ) {
			cout << "Please perform sync gesture (wave out)." << endl;
			t1 = steady_clock::now();
		}
		
	}
	cout << "Sync Complete!" << endl;

	/*///------------------------------------------------------------------------------------------------
	Connect to the Tensorflow server
	/*///------------------------------------------------------------------------------------------------
	cout << "Establishing connection to TensorFlow server" << endl;
	WSADATA wsaData;
	SOCKET ConnectSocket = INVALID_SOCKET;
	std::clock_t    start;
	struct addrinfo *result = NULL, *ptr = NULL, hints;
	const char *sendbuf = "10|15|-103|-66|-23|-14|-11|12|-7|-18|38|17|7|5|4|13|-6|-38|-17|-7|3|0|5|11|-6|-38|-17|-7|3|0|5|11|-14|-53|-86|-52|-18|-10|-11|-14|9|50|45|32|13|5|-10|-7|13|22|-34|13|6|2|-3|4|17|0|3|-6|0|4|26|-2|-25|19|80|12|0|-3|-10|-24|17|0|25|1|4|4|3|-17|-18|-37|15|10|4|-1|-12|10|11|101|44|28|9|3|9|21|10|44|-7|18|8|7|-5|14|-12|-55|-94|-7|-3|-4|-1|9|12|6|35|0|-6|-9|11|-11|3|-60|2|-25|-10|0|58|-33|-15|34|8|9|-2|-1|-6|2|-5|34|-25|-9|2|2|-14|-5|9|-41|48|-19|-5|0|13|6|-18|-7|38|13|5|0|7|-14|7|16|44|-3|-1|2|5|19|-15|-21|-112|-43|-14|-6|-18|12|-13|-20|-11|-15|-1|0|-8|-2|32|-31|73|59|19|9|-1|6|19|-2|9|41|9|3|-59|20|-3|-22|-76|-53|-17|-9|-9|-34|14|18|68|20|11|3|13|55|-37|3|101|29|2|-5|-13|-12|-15|-35|-52|-15|-5|-2|17|-20|3|49|15|-6|-8|-6|-6|20|3|49|15|-6|-8|-6|-6|20|4|22|9|-2|-2|3|10|-15|-23|-46|-42|0|0|-4|-19|-10|15|-2|2|-6|-9|-5|14|-7|-5|31|-43|-4|3|2|-1|-24|-5|31|-43|-4|3|2|-1|-24|-1|-30|-12|-16|-2|-5|-32|-5|-5|-16|13|23|6|1|8|5|-5|-16|13|23|6|1|8|5|5|-8|80|38|11|6|7|5|5|-8|80|38|11|6|7|5|-11|-31|-9|-10|-4|-6|25|-6|-11|-31|-9|-10|-4|-6|25|-6|20|-8|30|-4|-4|-1|-6|9|-6|-22|-13|-10|-5|-1|2|15|11|13|-6|-5|-1|-1|2|-4|11|13|-6|-5|-1|-1|2|-4|2|69|54|16|3|1|-16|-1|-7|67|-27|-12|2|1|2|1|-7|67|-27|-12|2|1|2|1|";
	int max = 127;
	int min = -128;

	char recvbuf[DEFAULT_NETWORK_BUFLEN];
	int iResult;
	int recvbuflen = DEFAULT_NETWORK_BUFLEN;


	// Initialize Winsock
	iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (iResult != 0) {
		printf("WSAStartup failed with error: %d\n", iResult);
		return 1;
	}

	ZeroMemory(&hints, sizeof(hints));
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;
	// Resolve the server address and port
	iResult = getaddrinfo("localhost", DEFAULT_NETWORK_PORT, &hints, &result);
	if (iResult != 0) {
		printf("getaddrinfo failed with error: %d\n", iResult);
		WSACleanup();
		return 1;
	}
	// Attempt to connect to an address until one succeeds
	while ((ConnectSocket == INVALID_SOCKET)) {
		for (ptr = result; ptr != NULL; ptr = ptr->ai_next) {


			// Create a SOCKET for connecting to server
			ConnectSocket = socket(ptr->ai_family, ptr->ai_socktype,
				ptr->ai_protocol);
			if (ConnectSocket == INVALID_SOCKET) {
				printf("socket failed with error: %ld\n", WSAGetLastError());
				WSACleanup();
				return 1;
			}
			start = std::clock();
			// Connect to server.
			iResult = connect(ConnectSocket, ptr->ai_addr, (int)ptr->ai_addrlen);
			std::cout << "Time: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;
			if (iResult == SOCKET_ERROR) {
				closesocket(ConnectSocket);
				ConnectSocket = INVALID_SOCKET;


				continue;
			}

		}
	}
	if (ConnectSocket == INVALID_SOCKET) {
		printf("Unable to FUCKYOU  connect to server!\n");
		WSACleanup();
		return 1;
	}

	freeaddrinfo(result);

	listener.synced = true;
	/*///------------------------------------------------------------------------------------------------
	Start control loop
	/*///------------------------------------------------------------------------------------------------
	using namespace std::chrono;
	t1 = steady_clock::now();
	steady_clock::time_point t2 = steady_clock::now();
	steady_clock::time_point t3 = steady_clock::now();
	size_t count = 0;
	while (true) {
		t3 = steady_clock::now();
		hub.runOnce(20);
		received = teensy.readSerialPortGood('\n');
		
		if ( duration_cast<milliseconds>(steady_clock::now() - t2).count() > 50 ) {
			std::string randEMG;
			for (size_t i = 0; i < 400; i++) {
				int randNum = listener.emg_data[i];
				std::string buf = std::to_string(randNum) + "|";
				randEMG += buf;
			}
			const char * randEMGCstr = randEMG.c_str();
			iResult = send(ConnectSocket, randEMGCstr, (int)strlen(randEMGCstr), 0);
			if (iResult == SOCKET_ERROR) {
				//printf("send failed with error: %d\n", WSAGetLastError());
				closesocket(ConnectSocket);
				WSACleanup();
				return 1;
			}


			while (iResult > 0) {
				//printf("Receiving:\n");
				iResult = recv(ConnectSocket, recvbuf, recvbuflen, 0);
				if (iResult > 0) {
					printf("Bytes received: %d\n", iResult);
					for (int i = 0; i < iResult; i++) {
						std::cout << recvbuf[i];
					}std::cout <<" ";
				}
				else if (iResult == 0) {
					//printf("###########################################\n");
				}
				else {
					//printf("recv failed with error: %d\n", WSAGetLastError());
				}

				break;
			};
			t2 = steady_clock::now();

			std::string AI_gesture = "";
			int k = 0;
			while ( (recvbuf[k] != '\0') && (recvbuf[k] != ',') ) {
				AI_gesture += recvbuf[k];
				k++;
			}
			if (AI_gesture == "Claw") {
				listener.pose_int = 5;
			}
			else if (AI_gesture == "Thumbsup") {
				listener.pose_int = 4;
			}
			else if (AI_gesture == "Resting") {
				listener.pose_int = 1;
			}
			else {
				listener.pose_int = 0;
			}

			cout << "Transmitting: " << +listener.yaw << " " << +listener.yaw_int << " " << listener.yaw_offset << " " << +listener.pitch_int << " " << +listener.roll_int << " " << +listener.pose_int << endl;
			
		}

		if (received.size() > 0) {
			string output = "";

			output += listener.yaw_int;
			output += listener.pitch_int;
			output += listener.roll_int;
			output += listener.pose_int;
			output += '\0';

			if (duration_cast<milliseconds>(steady_clock::now() - t1).count() > 10) {
				bool transmitted = teensy.writeSerialPortGood(output, '\n');
				
				//cout << "Received: " << received << endl;
				//cout << endl;
				t1 = steady_clock::now();
			}
			
		}
		this_thread::yield();
	
	}


	while (true);
}

