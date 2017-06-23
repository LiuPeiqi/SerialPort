#include "SerialPort/SerialPort.h"
#include <chrono>
#include <iostream>
using namespace std;
int main()
{
	auto coms = lpq::ListSerialPorts();
	return 0;
}
