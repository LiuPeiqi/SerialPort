#ifndef LPQ_LIB_SERIAL_PORT_H
#define LPQ_LIB_SERIAL_PORT_H
#include <chrono>
#include <exception>
#include <stdexcept>
#include <functional>
#include <string>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <boost/asio.hpp>

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
		class TimeoffError : public std::runtime_error
		{
		public: TimeoffError(const std::string& msg) :std::runtime_error(msg) {}
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
			read_thread = std::make_unique<std::thread>(std::bind(std::mem_fn(&SerialPort::ReceiveThread), this));
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
				std::lock_guard<std::mutex>(*serial_port_lock);
				*break_thread_flag = true;
				read_thread->join();
				prop->port.close();
			}
		}
		SerialPort(const SerialPort&) = delete;
		SerialPort& operator=(const SerialPort&) = delete;

		size_t Send(const std::string& msg);
		std::string Receive(size_t byte_count);
		std::string ReceiveUntil(size_t byte_count, const std::string& termainal);
		std::string Demand(const std::string& cmd, const std::string& confirm_regex);
		using ConfirmCallBack = std::function<bool(const char* buf_beg, const char* buf_end)>;
		std::string Demand(const std::string& cmd, ConfirmCallBack confirm);

		void ResponseTimeout(double timeout);
		double ResponseTimeout() const;
		void BufferSize(size_t buffer_size);
		size_t BufferSize() const;
		void Override(bool flag);
		bool Overrider() const;
		bool HadOverride();
		void Terminal(const std::string& t);
		const std::string& Terminal() const;
	private:
		struct Property{
			boost::asio::io_service io;
			boost::asio::serial_port port;
			std::string port_name;
			std::string terminal;
			std::unique_ptr<char[]> buffer;
			size_t buffer_size = 1024;
			size_t write_cycle = 0;
			size_t read_cycle = 0;
			char const* read_buf_beg = nullptr;
			char const* buf_end = nullptr;
			char * buf_cur = nullptr;
			bool overide_flag = true;
			Property(const std::string& com, unsigned int baud_rate, StopBits stop_bits,
				Parity parity, FlowControl flow_control, unsigned int data_bits):
				io(), port(io, com), port_name(com)
			{
				port.set_option(boost::asio::serial_port::baud_rate(baud_rate));
				port.set_option(boost::asio::serial_port::stop_bits(stop_bits));
				port.set_option(boost::asio::serial_port::parity(parity));
				port.set_option(boost::asio::serial_port::flow_control(flow_control));
				port.set_option(boost::asio::serial_port::character_size(data_bits));
				io.run();

				buffer = std::unique_ptr<char[]>(new char[buffer_size]);
				buf_cur = buffer.get();
				read_buf_beg = buf_cur;
				buf_end = read_buf_beg + buffer_size;
			}
		};
		std::unique_ptr<std::mutex> serial_port_lock;
		std::unique_ptr<Property> prop;
		std::unique_ptr<std::thread> read_thread;
		std::unique_ptr<bool> break_thread_flag;
		std::chrono::milliseconds timeout = std::chrono::milliseconds(20);
		void ReceiveThread()
		{
			// store local pointe to access resources when move construction occurred.
			std::mutex* const sp_lock = this->serial_port_lock.get();
			Property* const prop = this->prop.get();
			bool * const break_flag = this->break_thread_flag.get();
			size_t async_receive_bytes = 0;
			bool async_read_complete = false;
			auto async_read_handler = [&](const boost::system::error_code& ec, size_t bytes)->void {
				if (ec) {
					//log and exit thread
					;
				}
				async_receive_bytes = bytes;
				async_read_complete = true;
			};
			while (true) {
				{
					std::lock_guard<std::mutex> sp_guard(*sp_lock);
					if (*break_flag) {
						break;
					}
					if (async_read_complete) {
						if ((prop->buf_cur + async_receive_bytes) >= prop->buf_end) {
							prop->buf_cur = prop->buffer.get();
							++prop->write_cycle;
						}
						else {
							prop->buf_cur += async_receive_bytes;
						}
						async_read_complete = false;
						async_receive_bytes = 0;
						const static size_t READ_SIZE = 128;
						size_t buffer_remain = prop->buf_end - prop->buf_cur;
						size_t max_read_size = buffer_remain < READ_SIZE ? buffer_remain : READ_SIZE;
						prop->port.async_read_some(boost::asio::buffer(reinterpret_cast<void*>(prop->buf_cur), max_read_size), async_read_handler);
					}
				}//release lock
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
			}
			std::lock_guard<std::mutex> sp_guard(*sp_lock);
			prop->port.cancel();
			return;
		}
	};
}
#endif // !LPQ_LIB_SERIAL_PORT_H
