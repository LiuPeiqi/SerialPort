#ifndef LPQ_LIB_SERIAL_PORT_H
#define LPQ_LIB_SERIAL_PORT_H
#include <chrono>
#include <exception>
#include <stdexcept>
#include <functional>
#include <string>
#include <regex>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <boost/asio.hpp>
#include <boost/circular_buffer.hpp>

#include <Windows.h>

namespace lpq {
	/*
	std::vector<std::string> ListSerialPorts();// list all computer com port.

	*/
	inline std::vector<std::string> ListSerialPorts()
	{
		HKEY hkey;
		auto result = ::RegOpenKeyEx(HKEY_LOCAL_MACHINE, "HARDWARE\\DEVICEMAP\\SERIALCOMM", 0, KEY_READ, &hkey);
		auto throw_except = [](DWORD ret_code) {
			if (ERROR_SUCCESS != ret_code) {
				throw std::runtime_error("Cannot find key 'HARDWARE\\DEVICEMAP\\SERIALCOMM'!");
			}
		};
		throw_except(result);
		DWORD key_count = 0;
		::RegQueryInfoKey(hkey, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, &key_count, nullptr, nullptr, nullptr, nullptr);
		std::vector<std::string> coms;
		for (DWORD i = 0; i < key_count; ++i) {
			static const size_t MAX_LEN = 256;
			char name[MAX_LEN];
			BYTE value[MAX_LEN];
			DWORD value_len = MAX_LEN, name_len = MAX_LEN;
			result = ::RegEnumValue(hkey, i, name, &name_len, nullptr, nullptr, value, &value_len);
			throw_except(result);
			coms.push_back(std::string(value, value + value_len));
		}
		return std::move(coms);
	}

	class SerialPort
	{
	public:
		using StopBits = boost::asio::serial_port::stop_bits::type;
		using Parity = boost::asio::serial_port::parity::type;
		using FlowControl = boost::asio::serial_port::flow_control::type;
		class TimeoutError : public std::runtime_error
		{
		public: TimeoutError(const std::string& msg) :std::runtime_error(msg) {}
		};
		class SendError :public std::runtime_error
		{
		public: SendError(const std::string& msg) :std::runtime_error(msg) {}
		};
		class ResponseError : public std::runtime_error
		{
		public:	ResponseError(const std::string& msg) :std::runtime_error(msg) {}
		};
		class BufferFullError : public std::runtime_error
		{
		public: BufferFullError(const std::string& msg) : std::runtime_error(msg) {}
		};

		SerialPort(const std::string& com_name,
			unsigned int baud_rate = 9600,
			StopBits stop_bits = StopBits::one,
			Parity parity = Parity::none,
			FlowControl flow_control = FlowControl::none,
			unsigned int data_bits = 8) :
			com(io_service, com_name),
			receive_buffer(boost::circular_buffer<char>(1024)),
			break_receive_flag(false)
		{
			std::lock_guard<std::mutex> construction_lock(serial_port_lock);
			com.set_option(boost::asio::serial_port::baud_rate(baud_rate));
			com.set_option(boost::asio::serial_port::stop_bits(stop_bits));
			com.set_option(boost::asio::serial_port::parity(parity));
			com.set_option(boost::asio::serial_port::flow_control(flow_control));
			com.set_option(boost::asio::serial_port::character_size(data_bits));

			std::thread read_thread(std::bind(&SerialPort::ReceiveThread, this));
			read_thread.detach();
			std::this_thread::sleep_for(timeout);
		}

		~SerialPort() {
			std::lock_guard<std::mutex> sp_lock(serial_port_lock);
			break_receive_flag = true;
			com.close();
		}

		SerialPort(SerialPort&& right) = delete;
		SerialPort(const SerialPort&) = delete;
		SerialPort& operator=(const SerialPort&) = delete;

		size_t Send(const std::string& msg) 
		{
			std::lock_guard<std::mutex> send_lock(serial_port_lock);
			return SendWithoutLock(msg);
		}

		std::string Receive(size_t byte_count)//return string.size() maybe not eq byte_count;
		{
			if (byte_count == 0) {
				return std::string();
			}
			auto receive_complete = [=](BufIter begin, BufIter end) {
				if (static_cast<size_t>(end - begin) >= byte_count) {
					return begin + byte_count;
				}
				else {
					return begin;
				}
			};
			return std::move(ReceiveUntil(receive_complete));
		}


		std::string ReceiveUntil(const std::string& termainal)
		{// return without termainal and maybe throw TimeoutError exception;
			auto search = [&](BufIter begin, BufIter end) {
				auto iter = std::search(begin, end, termainal.cbegin(), termainal.cend());
				if (iter == end) {
					return begin;
				}
				iter += termainal.size();
				return iter;
			};
			return std::move(ReceiveUntil(search));
		}

		void CleanReceiveBuffer()
		{
			std::lock_guard<std::mutex> clean_lock(serial_port_lock);
			receive_buffer.erase_begin(receive_buffer.size());
		}

		std::string Demand(const std::string& cmd, std::string confirm_regex, bool ignore_case = false)
		{//if not confirm whith confirm_regex, only throw TimeoutError.
			if ((confirm_regex.size() > 0) && confirm_regex[0] != '^') {
				confirm_regex.insert(confirm_regex.begin(), '^');
			}
			{
				std::lock_guard<std::mutex> clean_buffer_lock(serial_port_lock);
				receive_buffer.erase_begin(receive_buffer.size());
				SendWithoutLock(cmd);
			}//release lock
			auto icase = std::regex::icase ^ std::regex::icase;
			if (ignore_case) {
				icase = std::regex::icase;
			}
			std::regex re_confirm(confirm_regex, icase);
			
			auto confirm = [&](BufIter begin, BufIter end) {
				std::match_results<BufIter> result;
				if (std::regex_search(begin, end, result, re_confirm)) {
					return begin + result[0].length();
				}
				else {
					return begin;
				}
			};
			try {
				return std::move(ReceiveUntil(confirm));
			}
			catch (TimeoutError&) {
				//log cmd, receive and confirm_regex;
				throw;
			}
			return std::string();//never run here;
		}

		//using ConfirmCallBack = std::function<BufIter(BufIter buf_beg, BufIter buf_end)>;
		//std::string Demand(const std::string& cmd, ConfirmCallBack confirm);

		void ResponseTimeout(double timeout)//second
		{
			std::lock_guard<std::mutex> lock(serial_port_lock);
			this->timeout = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(timeout));
		}
		double ResponseTimeout() const;

		void Override(bool flag);
		bool Overrider() const;
		void Terminal(const std::string& t)
		{
			std::lock_guard<std::mutex> set_lock(serial_port_lock);
			terminal = t;
		}

		const std::string& Terminal() const
		{
			return terminal;
		}
	private:
			void ReceiveThread()
			{
				const static size_t READ_SIZE = 128;
				char copy_buf[READ_SIZE];
				while (!break_receive_flag) {
					boost::system::error_code ec;
					auto receive_bytes = com.read_some(boost::asio::buffer(copy_buf, READ_SIZE), ec);
					if (ec) {
						if (ec.value() == 995) {
							//exit
						}
						else {
							//log error
						}
						break;
					}
					if(receive_bytes != 0){
						std::lock_guard<std::mutex> lock(serial_port_lock);
						std::copy(copy_buf, copy_buf + receive_bytes, std::back_inserter(receive_buffer));
						receive_bytes = 0;
					}//release lock
					std::this_thread::sleep_for(INTERVAL);
				}
				break_receive_flag = true;
				return;
			}

		std::mutex serial_port_lock;
		std::chrono::milliseconds timeout = std::chrono::milliseconds(20);
		boost::asio::io_service io_service;
		boost::asio::serial_port com;
		boost::circular_buffer<char> receive_buffer;
		std::string terminal;
		volatile bool break_receive_flag;
		static constexpr std::chrono::milliseconds INTERVAL = std::chrono::milliseconds(1);

		size_t SendWithoutLock(const std::string& msg)
		{
			if (break_receive_flag) {
				throw SendError("SerialPort has closed!");
			}
			auto byte = com.write_some(boost::asio::buffer(msg));
			auto check_size = msg.size();
			if (!terminal.empty()) {
				check_size += terminal.size();
				byte += com.write_some(boost::asio::buffer(terminal));
			}
			if (byte != check_size) {
				throw SendError("Send byte != cmd.size:\"" + msg + "\"" + std::to_string(byte) + "Bytes!");
			}
			return byte;
		}
		
		using BufIter = boost::circular_buffer<char>::iterator;

		std::string ReceiveUntil(std::function<BufIter(BufIter begin, BufIter end)> predicate)
		{
			if (break_receive_flag) {
				throw ResponseError("SerialPort has closed!");
			}
			size_t last_size = 0;
			std::string buf;
			auto start = std::chrono::system_clock::now();
			size_t count = 0;
			while (true) {
				{
					std::lock_guard<std::mutex> lock(serial_port_lock);
					if (0 == count++) {
						start = std::chrono::system_clock::now();
					}
					auto end_iter = predicate(receive_buffer.begin(), receive_buffer.end());
					if (end_iter != receive_buffer.begin()) {
						buf.assign(receive_buffer.begin(), end_iter);
						receive_buffer.erase(receive_buffer.begin(), end_iter);
						return std::move(buf);
					}
					auto step = std::chrono::system_clock::now();
					auto elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(step - start);
					if (last_size == receive_buffer.size() && elapsed_time > timeout) {
						receive_buffer.erase_begin(receive_buffer.size());
						throw TimeoutError("ReceiveUntil have no more new bytes in " + std::to_string(timeout.count()) + "ms!");
					}
					if (last_size != receive_buffer.size()) {
						last_size = receive_buffer.size();
						start = step;
					}
				}// release lock
				std::this_thread::sleep_for(INTERVAL);
			}
			return buf;// never run here;
		}
	};
}
#endif // !LPQ_LIB_SERIAL_PORT_H
