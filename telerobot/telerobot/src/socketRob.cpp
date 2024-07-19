#include "socketRob.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <streambuf>
#include <istream>
#include <ostream>

namespace galik { namespace net {

template<typename Char>
class basic_socketbuf
: public std::basic_streambuf<Char>
{
public:
	typedef Char char_type;
	typedef std::basic_streambuf<char_type> buf_type;
	typedef std::basic_ostream<char_type> stream_type;
	typedef typename buf_type::int_type int_type;
	typedef typename std::basic_streambuf<Char>::traits_type traits_type;

protected:

	static const int char_size = sizeof(char_type);
	static const int SIZE = 128;
	char_type obuf[SIZE] ;
	char_type ibuf[SIZE] ;

	int sock;

public:
	basic_socketbuf(): sock(0)
	{
		buf_type::setp(obuf, obuf + (SIZE - 1));
		buf_type::setg(ibuf, ibuf, ibuf);
	}

	virtual ~basic_socketbuf() { sync(); }

	void set_socket(int sock) { this->sock = sock; }
	int get_socket() { return this->sock; }

protected:

	int output_buffer()
	{
		int num = buf_type::pptr() - buf_type::pbase();
		if(send(sock, reinterpret_cast<char*>(obuf), num * char_size, 0) != num)
			return traits_type::eof();
		buf_type::pbump(-num);
		return num;
	}

	virtual int_type overflow(int_type c)
	{
		if(c != traits_type::eof())
		{
			*buf_type::pptr() = c;
			buf_type::pbump(1);
		}

		if(output_buffer() == traits_type::eof())
			return traits_type::eof();
		return c;
	}

	virtual int sync()
	{
		if(output_buffer() == traits_type::eof())
			return traits_type::eof();
		return 0;
	}

	virtual int_type underflow()
	{
		if(buf_type::gptr() < buf_type::egptr())
			return *buf_type::gptr();

		int num;
		if((num = recv(sock, reinterpret_cast<char*>(ibuf), SIZE * char_size, 0)) <= 0)
			return traits_type::eof();

		buf_type::setg(ibuf, ibuf, ibuf + num);
		return *buf_type::gptr();
	}
};

typedef basic_socketbuf<char> socketbuf;
typedef basic_socketbuf<wchar_t> wsocketbuf;

template<typename Char>
class basic_socketstream
: public std::basic_iostream<Char>
{
public:
	typedef Char char_type;
	typedef std::basic_iostream<char_type> stream_type;
	typedef basic_socketbuf<char_type> buf_type;

protected:
	buf_type buf;

public:
	basic_socketstream(): stream_type(&buf) {}
	basic_socketstream(int s): stream_type(&buf) { buf.set_socket(s); }

	void close()
	{
		if(buf.get_socket() != 0) ::close(buf.get_socket());
		stream_type::clear();
	}

	bool open(const std::string& host, uint16_t port)
	{
		close();
		int sd = socket(AF_INET, SOCK_STREAM, 0);
		sockaddr_in sin;
		hostent *he = gethostbyname(host.c_str());

		std::copy(reinterpret_cast<char*>(he->h_addr)
			, reinterpret_cast<char*>(he->h_addr) + he->h_length
			, reinterpret_cast<char*>(&sin.sin_addr.s_addr));
		sin.sin_family = AF_INET;
		sin.sin_port = htons(port);

		if(connect(sd, reinterpret_cast<sockaddr*>(&sin), sizeof(sin)) < 0)
			stream_type::setstate(std::ios::failbit);
		else
			buf.set_socket(sd);
		return *this;
	}
};

typedef basic_socketstream<char> socketstream;
typedef basic_socketstream<wchar_t> wsocketstream;

}}

galik::net::socketstream ss;



/*
 * Comunicates with Robot or Simulator.
 *
 */

socketRob::socketRob(std::string hostName, int port) {

	ss.open(hostName, port);

}

void socketRob::sendJointsSimulator(Eigen::VectorXd angles) {

    int i;
    for( i=0; i<angles.size()-1; i++ ){
        ss << angles(i) << ",";
    }

    ss << angles(angles.size()-1);

    // ss << angles(angles.size()-1) << ";";
	
    ss << std::flush;

}

void socketRob::sendString(std::string str) {

	ss << str;  	
	ss << std::flush;

}

void socketRob::sendJointsKuka(Eigen::VectorXd angles) {
	ss << "SJ ";
    ss << angles(0) << " ";
    ss << angles(1) <<" ";
    ss << angles(2) <<" ";
    ss << angles(3) <<" ";
	ss << 0.0 <<" ";
	ss << 0.0 <<" ";
    ss << 0.0 <<" ";
	
	ss << std::flush;

}

