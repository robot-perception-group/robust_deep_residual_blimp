#include "boost/asio.hpp"
#include "boost/smart_ptr.hpp"
#include <boost/lexical_cast.hpp>
#include <string>

#define READBUFFER 10240
class readbuffer_ {
public:
    uint8_t data[READBUFFER];
    const size_t length = READBUFFER;
    size_t fill   = 0;
    size_t offset = 0;
};

class anonymoussocket {
public:
    enum type { _UNSET, _SERIAL, _UDP } type_;

    boost::shared_ptr<boost::asio::serial_port> serial;
    boost::shared_ptr<boost::asio::ip::udp::socket> udp;
    boost::asio::io_service io_service;
    boost::asio::ip::udp::endpoint remote;
    boost::shared_ptr<readbuffer_> readbuffer;

    anonymoussocket()
    {
        type_ = _UNSET;
    }

    void open_udp(std::string server, std::string port)
    {
        type_      = _UDP;
        udp        = boost::shared_ptr<boost::asio::ip::udp::socket>(new boost::asio::ip::udp::socket(io_service));
        readbuffer = boost::shared_ptr<readbuffer_>(new readbuffer_());
        remote     = boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(server), boost::lexical_cast<int>(port));
        udp->open(boost::asio::ip::udp::v6());
    }
    void open_serial(std::string device, std::string baud_rate)
    {
        type_  = _SERIAL;
        serial = boost::shared_ptr<boost::asio::serial_port>(new boost::asio::serial_port(io_service));
        serial->open(device);
        serial->set_option(boost::asio::serial_port_base::baud_rate(boost::lexical_cast<int>(baud_rate)));
    }

    void close()
    {
        switch (type_) {
        case _UDP:
            udp->close();
            udp.reset();
            readbuffer.reset();
            type_ = _UNSET;
            break;
        case _SERIAL:
            serial->close();
            serial.reset();
            type_ = _UNSET;
            break;
        default:
            break;
        }
    }

    ~anonymoussocket()
    {
        close();
    }

    int write(uint8_t *buffer, size_t length)
    {
        int res;

        switch (type_) {
        case _UDP:
            res = udp->send_to(boost::asio::buffer(buffer, length), remote);
            break;
        case _SERIAL:
            res = boost::asio::write(*serial, boost::asio::buffer(buffer, length));
            break;
        default:
            res = -1;
            break;
        }
        return res;
    }


    int safereadudp(uint8_t *buffer, size_t length)
    {
        size_t res;

        if (readbuffer->fill == 0) {
            res = udp->receive_from(boost::asio::buffer(readbuffer->data, readbuffer->length), remote);
            if (res > 0) {
                readbuffer->fill   = res;
                readbuffer->offset = 0;
            } else {
                return res;
            }
        }
        if (readbuffer->fill > 0) {
            size_t r = readbuffer->fill;
            if (r > length) {
                r = length;
            }
            memcpy(buffer, &readbuffer->data[readbuffer->offset], r);
            readbuffer->fill   -= r;
            readbuffer->offset += r;
            return r;
        } else {
            return 0;
        }
    }

    int read(uint8_t *buffer, size_t length)
    {
        int res;

        switch (type_) {
        case _UDP:
            res = safereadudp(buffer, length);
            break;
        case _SERIAL:
            res = boost::asio::read(*serial, boost::asio::buffer(buffer, length));
            break;
        default:
            res = -1;
            break;
        }
        return res;
    }
};
