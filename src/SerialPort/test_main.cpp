#include "SerialPort/SerialPort.h"
#include <chrono>
#include <iostream>
using namespace std;
int main()
{
	auto coms = lpq::ListSerialPorts();
	try {
		lpq::SerialPort serial_port(coms[1]);
		serial_port.Terminal("\r\n");
		serial_port.Send("*idn?");
		while (true) {
			cout << serial_port.ReceiveUntil("\r\n") << endl;
		}
	}
	catch (exception& e) {
		cerr << e.what();
	}
	return 0;
}
