#include <string>
#include "SerialPort.h"

SerialPort::SerialPort(const char *portName)
{
	this->connected = false;

	this->handler = CreateFileA(static_cast<LPCSTR>(portName),
		GENERIC_READ | GENERIC_WRITE,
		0,
		NULL,
		OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL,
		NULL);
	if (this->handler == INVALID_HANDLE_VALUE) {
		if (GetLastError() == ERROR_FILE_NOT_FOUND) {
			printf("ERROR: Handle was not attached. Reason: %s not available\n", portName);
		}
		else
		{
			printf("ERROR!!!");
		}
	}
	else {
		DCB dcbSerialParameters = { 0 };

		if (!GetCommState(this->handler, &dcbSerialParameters)) {
			printf("failed to get current serial parameters");
		}
		else {
			dcbSerialParameters.BaudRate = CBR_9600;
			dcbSerialParameters.ByteSize = 8;
			dcbSerialParameters.StopBits = ONESTOPBIT;
			dcbSerialParameters.Parity = NOPARITY;
			dcbSerialParameters.fDtrControl = DTR_CONTROL_ENABLE;

			if (!SetCommState(handler, &dcbSerialParameters))
			{
				printf("ALERT: could not set Serial port parameters\n");
			}
			else {
				this->connected = true;
				PurgeComm(this->handler, PURGE_RXCLEAR | PURGE_TXCLEAR);
				Sleep(ARDUINO_WAIT_TIME);
			}
		}
	}
}

SerialPort::~SerialPort()
{
	if (this->connected) {
		this->connected = false;
	}
}

int SerialPort::readSerialPort(char *buffer, unsigned int buf_size)
{
	DWORD bytesRead;
	unsigned int toRead = 0;

	ClearCommError(this->handler, &this->errors, &this->status);

	if (this->status.cbInQue > 0) {
		if (this->status.cbInQue > buf_size) {
			toRead = buf_size;
		}
		else toRead = this->status.cbInQue;
	}
	/*
		memset(buffer, 0, buf_size);*/

	if (ReadFile(this->handler, buffer, toRead, &bytesRead, NULL)) {
		return bytesRead;
	} else {
		return -1;
	}
}

bool SerialPort::writeSerialPort(const char *buffer, unsigned int buf_size)
{
	DWORD bytesSend;

	if (!WriteFile(this->handler, (void*)buffer, buf_size, &bytesSend, 0)) {
		ClearCommError(this->handler, &this->errors, &this->status);
		return false;
	}
	else return true;
}
//bool SerialPort::writeSerialPort(std::string buffer, unsigned int buf_size)
//{
//	DWORD bytesSend;
//
//	if (!WriteFile(this->handler, (void*)buffer, buf_size, &bytesSend, 0)) {
//		ClearCommError(this->handler, &this->errors, &this->status);
//		return false;
//	}
//	else return true;
//}
bool SerialPort::writeSerialPort(char *buffer, unsigned int buf_size)
{
	DWORD bytesSend;

	if (!WriteFile(this->handler, (void*)buffer, buf_size, &bytesSend, 0)) {
		ClearCommError(this->handler, &this->errors, &this->status);
		return false;
	}
	else return true;
}
bool SerialPort::writeSerialPortGood(std::string input, char term)
{
	std::string termInput = input + term;
	size_t buff_size = termInput.size();
	char buffer[255];
	for (size_t i = 0; i<buff_size; i++) {
		buffer[i] = termInput[i];
	}

	DWORD bytesSend;
	ClearCommError(this->handler, &this->errors, &this->status);
	if (!WriteFile(this->handler, (void*)buffer, buff_size, &bytesSend, 0)) {
		ClearCommError(this->handler, &this->errors, &this->status);
		return false;
	}
	else {
		return true;
	}
}
std::string SerialPort::readSerialPortGood(char term)
{
	char buffer[2];
	std::string output;
	bool failTerm = false;
	char lastChar = ' ';
	while (lastChar != term) {
		int inputSize = readSerialPort(buffer, 1);
		if (inputSize > 0) {
			lastChar = buffer[0];
			output += lastChar;
		}
		else {
			failTerm = true;
			break;
		}
	}
	if (!failTerm) {
		output.pop_back();
	}
	return output;
}
bool SerialPort::isConnected()
{
	return this->connected;
}