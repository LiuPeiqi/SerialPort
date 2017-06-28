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

		SerialPort(const std::string& com,
			unsigned int baud_rate = 9600,
			StopBits stop_bits = StopBits::one,
			Parity parity = Parity::none,
			FlowControl flow_control = FlowControl::none,
			unsigned int data_bits = 8) :
			serial_port_lock(std::make_unique<std::mutex>()),
			break_thread_flag(std::make_unique<bool>(false))
		{
			std::lock_guard<std::mutex> construction_lock(*serial_port_lock);
			prop = std::make_unique<Property>(com, baud_rate, stop_bits, parity, flow_control, data_bits);
			read_thread = std::make_unique<std::thread>(
				std::bind(
					std::mem_fn(&SerialPort::Property::ReceiveThread), prop.get(), serial_port_lock.get(),break_thread_flag.get()));
		}
		SerialPort(SerialPort&& right)
		{
			std::lock_guard<std::mutex>(*right.serial_port_lock);
			prop = std::move(right.prop);
			read_thread = std::move(right.read_thread);
			break_thread_flag = std::move(right.break_thread_flag);
			timeout = std::move(right.timeout);
			serial_port_lock = std::move(right.serial_port_lock);
		}
		~SerialPort() {
			if (prop != nullptr) {
				std::lock_guard<std::mutex> sp_lock(*serial_port_lock);
				*break_thread_flag = true;
				read_thread->join();
				prop->port.close();
			}
		}
		SerialPort(const SerialPort&) = delete;
		SerialPort& operator=(const SerialPort&) = delete;

		size_t Send(const std::string& msg) 
		{
			std::lock_guard<std::mutex> send_lock(*serial_port_lock);
			return SendWithoutLock(msg);
		}

		std::string Receive(size_t byte_count)//return string.size() maybe not eq byte_count;
		{
			auto receive_complete = [=](BufIter begin, BufIter end) {
				if (static_cast<size_t>(end - begin) >= byte_count) {
					return begin + byte_count;
				}
				else {
					return end;
				}
			};
			return std::move(ReceiveUntil(receive_complete));
		}


		std::string ReceiveUntil(const std::string& termainal)
		{// return without termainal and maybe throw TimeoutError exception;
			auto search = [&](BufIter begin, BufIter end) {
				auto iter = std::search(begin, end, termainal.cbegin(), termainal.cend());
				if (iter == end) {
					return end;
				}
				iter += termainal.size();
				return iter;
			};
			return std::move(ReceiveUntil(search));
		}

		void CleanReceiveBuffer()
		{
			std::lock_guard<std::mutex> clean_lock(*serial_port_lock);
			prop->buffer.erase_begin(prop->buffer.size());
		}

		std::string Demand(const std::string& cmd, const std::string& confirm_regex)
		{//if not confirm whith confirm_regex, only throw TimeoutError.
			{
				std::lock_guard<std::mutex> clean_buffer_lock(*serial_port_lock);
				prop->buffer.erase_begin(prop->buffer.size());
				SendWithoutLock(cmd);
			}//release lock
			std::regex re_confirm(confirm_regex);
			auto confirm = [&](BufIter begin, BufIter end) {
				std::match_results<BufIter> result;
				if (std::regex_match(begin, end, result, re_confirm)) {
					return begin + result[0].length();
				}
				else {
					return end;
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

		void ResponseTimeout(double timeout);
		double ResponseTimeout() const;
		void BufferSize(size_t buffer_size);

		size_t BufferSize() const { return prop->buffer_size; }

		void Override(bool flag);
		bool Overrider() const;
		void Terminal(const std::string& t)
		{
			std::lock_guard<std::mutex> set_lock(*serial_port_lock);
			this->prop->terminal = t;
		}

		const std::string& Terminal()
		{
			std::lock_guard<std::mutex> set_lock(*serial_port_lock);
			return this->prop->terminal;
		}
	private:
		struct Property{
			boost::asio::io_service io;
			boost::asio::serial_port port;
			std::string port_name;
			std::string terminal;
			boost::circular_buffer<char> buffer;
			size_t buffer_size = 1024;

			Property(const std::string& com, unsigned int baud_rate, StopBits stop_bits,
				Parity parity, FlowControl flow_control, unsigned int data_bits):
				io(), port(io, com), port_name(com)
			{
				port.set_option(boost::asio::serial_port::baud_rate(baud_rate));
				port.set_option(boost::asio::serial_port::stop_bits(stop_bits));
				port.set_option(boost::asio::serial_port::parity(parity));
				port.set_option(boost::asio::serial_port::flow_control(flow_control));
				port.set_option(boost::asio::serial_port::character_size(data_bits));
				
				buffer = boost::circular_buffer<char>(buffer_size);
			}

			void ReceiveThread(std::mutex* write_lock, volatile bool* break_flag)
			{
				volatile size_t async_receive_bytes = 0;
				volatile bool async_read_complete = true;
				auto async_read_handler = [&](const boost::system::error_code& ec, size_t bytes)->void {
					if (ec) {
						//log and exit thread
						;
					}
					async_receive_bytes = bytes;
					async_read_complete = true;
				};
				const static size_t READ_SIZE = 128;
				char copy_buf[READ_SIZE];
				while (!*break_flag) {
					if (async_read_complete) {
						std::lock_guard<std::mutex> sp_guard(*write_lock);
						std::copy(copy_buf, copy_buf + async_receive_bytes, std::back_inserter(buffer));
						async_read_complete = false;
						async_receive_bytes = 0;
						port.async_read_some(boost::asio::buffer(copy_buf, READ_SIZE), async_read_handler);
						io.run();
					}//release lock
					std::this_thread::sleep_for(INTERVAL);
				}
				port.cancel();
				return;
			}
		};


		std::unique_ptr<std::mutex> serial_port_lock;
		std::unique_ptr<Property> prop;
		std::unique_ptr<std::thread> read_thread;
		std::unique_ptr<volatile bool> break_thread_flag;
		std::chrono::milliseconds timeout = std::chrono::milliseconds(20000);
		static constexpr std::chrono::milliseconds INTERVAL = std::chrono::milliseconds(20);

		size_t SendWithoutLock(const std::string& msg)
		{
			auto byte = prop->port.write_some(boost::asio::buffer(msg));
			auto check_size = msg.size();
			if (!prop->terminal.empty()) {
				check_size += prop->terminal.size();
				byte += prop->port.write_some(boost::asio::buffer(prop->terminal));
			}
			if (byte != check_size) {
				throw SendError("Send byte != cmd.size:\"" + msg + "\"" + std::to_string(byte) + "Bytes!");
			}
			return byte;
		}
		
		using BufIter = boost::circular_buffer<char>::iterator;

		std::string ReceiveUntil(std::function<BufIter(BufIter begin, BufIter end)> predicate)
		{
			auto start = std::chrono::system_clock::now();
			size_t last_size = 0;
			std::string buf;
			while (true) {
				{
					std::lock_guard<std::mutex> read_lock(*serial_port_lock);
					auto end_iter = predicate(prop->buffer.begin(), prop->buffer.end());
					if (end_iter != prop->buffer.end()) {
						buf.assign(prop->buffer.begin(), end_iter);
						prop->buffer.erase(prop->buffer.begin(), end_iter);
						return std::move(buf);
					}
					auto step = std::chrono::system_clock::now();
					auto elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(step - start);
					if (elapsed_time > timeout) {
						throw TimeoutError("ReceiveUntil have no more new bytes in " + std::to_string(timeout.count()) + "ms!");
					}
					if (last_size != prop->buffer.size()) {
						last_size = prop->buffer.size();
						start = step;
					}
				}// release lock
				std::this_thread::sleep_for(INTERVAL);
			}
			return buf;// never run here;
		}

		void ReceiveThread()
		{
			// store local pointe to access resources when move construction occurred.
			std::mutex* const sp_lock = this->serial_port_lock.get();
			Property* const prop = this->prop.get();
			volatile bool * const break_flag = this->break_thread_flag.get();
			volatile size_t async_receive_bytes = 0;
			volatile bool async_read_complete = true;
			auto async_read_handler = [&](const boost::system::error_code& ec, size_t bytes)->void {
				if (ec) {
					//log and exit thread
					;
				}
				async_receive_bytes = bytes;
				async_read_complete = true;
				prop->io.run();
			};
			const static size_t READ_SIZE = 128;
			char copy_buf[READ_SIZE];
			while (!*break_flag) {
				if(async_read_complete){
					std::lock_guard<std::mutex> sp_guard(*sp_lock);
					std::copy(copy_buf, copy_buf + async_receive_bytes, std::back_inserter(prop->buffer));
					async_read_complete = false;
					async_receive_bytes = 0;
					prop->port.async_read_some(boost::asio::buffer(copy_buf, READ_SIZE), async_read_handler);
				}//release lock
				std::this_thread::sleep_for(INTERVAL);
			}
			prop->port.cancel();
			return;
		}
	};
}
#endif // !LPQ_LIB_SERIAL_PORT_H
