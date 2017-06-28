#include "SerialPort/SerialPort.h"
#include <chrono>
#include <iostream>
#include <sstream>
#include <thread>
#include <vector>
using namespace std;
int main()
{
	auto coms = lpq::ListSerialPorts();
	vector<size_t> slots928;
	vector<size_t> slots911;
	try {
		lpq::SerialPort serial_port(coms[1], 115200);
		serial_port.ResponseTimeout(0.1);
		serial_port.Terminal("\r\n");
		serial_port.Send("Exit");
		serial_port.Send("*rst");
		for (size_t i = 1; i < 9; ++i) {
			stringstream sst;
			sst << "GETN? " << i << ", 1024";
			serial_port.Send(sst.str());
			sst.str("");
			this_thread::sleep_for(1s);
			serial_port.CleanReceiveBuffer();
			sst << "SNDT " << i << ", '*rst'" << endl;
			serial_port.Send(sst.str());
			sst.str("");
			sst << "SNDT " << i << ", '*idn?'" << endl;
			serial_port.Send(sst.str());
			sst.str("");
			this_thread::sleep_for(1s);
			sst << "GETN?  " << i << ", 100" << endl;
			serial_port.Send(sst.str());
			auto idn = serial_port.ReceiveUntil("\r\n");
			serial_port.CleanReceiveBuffer();
			if (idn.size() > 7) {
				cout << i << ": " << idn;
			}
			if (idn.find("928") != string::npos) {
				slots928.push_back(i);
				/*sst.str("");
				sst << "SNDT " << i << ",\"BAUD 156250\"";
				serial_port.Send(sst.str());
				this_thread::sleep_for(1s);
				sst.str("");
				sst << "BAUD " << i << ",156250";
				serial_port.Send(sst.str());
				serial_port.Send("*idn?");
				auto res = serial_port.ReceiveUntil("\r\n");
				cout << res;
				sst.str("");
				sst << "BAUD? " << i;
				serial_port.Send(sst.str());
				res = serial_port.ReceiveUntil("\r\n");
				cout << res;
				this_thread::sleep_for(1s);
				sst.str("");
				sst << "CONN " << i << ",'Exit'";
				serial_port.Send(sst.str());
				serial_port.Send("*idn?");
				res = serial_port.ReceiveUntil("\r\n");
				cout << res;
				serial_port.Send("Exit");*/
				sst.str("");
				sst << "SNDT " << i << ", '*idn?'";
				serial_port.Send(sst.str());
				sst.str("");
				this_thread::sleep_for(1s);
				sst << "GETN? " << i << ",100";
				serial_port.Send(sst.str());
				idn = serial_port.Receive(5);
				size_t bytes = stoi(string(idn.begin() + 2, idn.end()));
				idn = serial_port.Receive(bytes);
				cout << idn;
				serial_port.CleanReceiveBuffer();
			}
			else if (idn.find("911") != string::npos) {
				slots911.push_back(i);
			}
		}
		serial_port.Demand("*idn?", "Stanford_Research_Systems,\\s*SIM900,\\s*s/n\\d+,\\s*ver\\d+(\\.\\d+)?\\r\\n", true);
		for (size_t i = 0; i < 1000; ++i) {
			auto start = chrono::system_clock::now();
			for (const auto& slot : slots928) {
				stringstream sst;
				sst << "CONN " << slot << ", \"Exit\"";
				serial_port.Send(sst.str());
				serial_port.Demand("*idn?", "Stanford_Research_Systems,\\s*SIM928,\\s*s/n\\d+,\\s*ver\\d+(\\.\\d+)?\\r\\n", true);
				serial_port.Demand("volt 1\r\nvolt?", "[\\+-]?\\d+(\\.\\d+)?\\r\\n");
				serial_port.Demand("opon\r\nexon?", "1\\r\\n");
				serial_port.Demand("volt 0\r\nvolt?", "[\\+-]?\\d+(\\.\\d+)?\\r\\n");
				serial_port.Demand("opof\r\nexon?", "0\\r\\n");
				serial_port.Send("Exit");
			}
			for (const auto& slot : slots911) {
				stringstream sst;
				sst << "CONN " << slot << ", \"Exit\"";
				serial_port.Send(sst.str());
				serial_port.Demand("*idn?", "stanford_research_systems,\\s*SIM911,\\s*s/n\\d+,\\s*ver\\d+(\\.\\d+)?\\r\\n", true);
				serial_port.Send("Exit");
				serial_port.Demand("*idn?", "Stanford_Research_Systems,\\s*SIM900,\\s*s/n\\d+,\\s*ver\\d+(\\.\\d+)?\\r\\n", true);
			}
			auto finish = chrono::system_clock::now();
			cout << chrono::duration_cast<chrono::milliseconds>(finish - start).count() << "ms" << endl;
		}
	}
	catch (exception& e) {
		cerr << e.what();
	}
	return 0;
}
