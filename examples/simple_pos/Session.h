#pragma once

#include <iostream>
#include <cassert>
#include <cstdint>
#include <vector>
#include <string>

#include <boost/array.hpp>

#include "common/boost_wrap.h"

#include "ublox/Message.h"
#include "ublox/message/NavPosllh.h"
#include "ublox/frame/UbloxFrame.h"

#include "comms/units.h"
#include "comms/process.h"

namespace ublox
{

namespace simple_pos    
{


class Session 
{
  protected:
    using InMessage =
        ublox::Message<
            comms::option::ReadIterator<const std::uint8_t*>,
            comms::option::Handler<Session> // Dispatch to this object
        >;


    using OutBuffer = std::vector<std::uint8_t>;
    using OutMessage =
        ublox::Message<
            comms::option::IdInfoInterface,
            comms::option::WriteIterator<std::back_insert_iterator<OutBuffer> >,
            comms::option::LengthInfoInterface
        >;
    using InNavAttMsg = ublox::message::NavAtt<InMessage>;
    using InNavPosllh = ublox::message::NavPosllh<InMessage>;
    using InEsfStatus = ublox::message::EsfStatus<InMessage>;
    using InEsfAlg =    ublox::message::EsfAlg<InMessage>;
    using InNavTimeutc =    ublox::message::NavTimeutc<InMessage>;
    using InNavSol =    ublox::message::NavSol<InMessage>;
    using InNavPvt =    ublox::message::NavPvt<InMessage>;


public:
    Session() = delete;
    Session(common::boost_wrap::io& io, const std::string& dev, unsigned int baudrate)
      : m_io(io),
        m_serial(io),
        m_pollTimer(io),
        m_device(dev),
        m_baudrate(baudrate)
    {
    }
    ~Session() = default;

    bool start()
    {
        boost::system::error_code ec;
        m_serial.open(m_device, ec);

        if (ec) {
            return false;
        }

        m_serial.set_option(SerialPort::baud_rate(m_baudrate));
        m_serial.set_option(SerialPort::character_size(8));
        m_serial.set_option(SerialPort::parity(SerialPort::parity::none));
        m_serial.set_option(SerialPort::stop_bits(SerialPort::stop_bits::one));
        m_serial.set_option(SerialPort::flow_control(SerialPort::flow_control::none));

        //configureUbxOutput();
        //sendPosPoll();
        performRead();
        return true;
    }


protected:

    using AllInMessages =
        std::tuple<
            InNavAttMsg,  InNavPvt, InEsfAlg
        >;



    using Frame = ublox::frame::UbloxFrame<InMessage, AllInMessages>;

    using SerialPort = boost::asio::serial_port;

    void performRead()
    {
        m_serial.async_read_some(
            boost::asio::buffer(m_inputBuf),
            [this](const boost::system::error_code& ec, std::size_t bytesCount)
            {
                if (ec == boost::asio::error::operation_aborted) {
                    return;
                }

                if (ec) {
                    std::cerr << "ERROR: read failed with message: " << ec.message() << std::endl;
                    m_io.stop();
                    return;
                }

                auto dataBegIter = m_inputBuf.begin();
                auto dataEndIter = dataBegIter + bytesCount;
                m_inData.insert(m_inData.end(), dataBegIter, dataEndIter);
                processInputData();
                performRead();
            });
    }
    void processInputData()
    {
        if (!m_inData.empty()) {
            auto consumed = comms::processAllWithDispatch(&m_inData[0], m_inData.size(), m_frame, *this);
            m_inData.erase(m_inData.begin(), m_inData.begin() + consumed);
        }    
    }

    void sendPosPoll()
    {
        using OutNavPosllhPoll = ublox::message::EsfAlgPoll<OutMessage>;
        sendMessage(OutNavPosllhPoll());

        m_pollTimer.expires_from_now(boost::posix_time::seconds(1));
        m_pollTimer.async_wait(
            [this](const boost::system::error_code& ec) {
                if (ec == boost::asio::error::operation_aborted) {
                    return;
                }

                sendPosPoll();
            });
    }

    void sendMessage(const OutMessage& msg)

    {

        OutBuffer buf;
        buf.reserve(m_frame.length(msg)); // Reserve enough space
        auto iter = std::back_inserter(buf);
        auto es = m_frame.write(msg, iter, buf.max_size());
        if (es == comms::ErrorStatus::UpdateRequired) {
            auto* updateIter = &buf[0];
            es = m_frame.update(updateIter, buf.size());
        }
        static_cast<void>(es);
        assert(es == comms::ErrorStatus::Success); // do not expect any error

        while (!buf.empty()) {
            boost::system::error_code ec;
            auto count = m_serial.write_some(boost::asio::buffer(buf), ec);

            if (ec) {
                std::cerr << "ERROR: write failed with message: " << ec.message() << std::endl;
                m_io.stop();
                return;
            }

            buf.erase(buf.begin(), buf.begin() + count);
        }
    }

    void configureUbxOutput()
    {
        using OutCfgPrtUsb = ublox::message::CfgPrtUsb<OutMessage>;

        OutCfgPrtUsb msg;
        auto& outProtoMaskField = msg.field_outProtoMask();

        using OutProtoMaskField = typename std::decay<decltype(outProtoMaskField)>::type;
        outProtoMaskField.setBitValue(OutProtoMaskField::BitIdx_outUbx, true);
        outProtoMaskField.setBitValue(OutProtoMaskField::BitIdx_outNmea, false);

        auto& inProtoMaskField = msg.field_inProtoMask();
        using InProtoMaskField = typename std::decay<decltype(inProtoMaskField)>::type;
        inProtoMaskField.setBitValue(InProtoMaskField::BitIdx_inUbx, true);
        inProtoMaskField.setBitValue(InProtoMaskField::BitIdx_inNmea, false);

        sendMessage(msg);
    }
  public:
    //void handle(InNavPosllh& msg){(void)msg; std::cout << "got1" << std::endl;}
    //void handle(InNavAttMsg& msg){(void)msg; std::cout << "got2" << std::endl;}
    //void handle(InEsfStatus& msg){(void)msg; std::cout << "got3" << std::endl;};
    //void handle(InEsfAlg& msg){(void)msg; std::cout << "got4" << std::endl; std::terminate();};
    //void handle(InNavTimeutc& msg){(void)msg; std::cout << "got5" << std::endl;};;
    //void handle(InNavSol& msg){(void)msg; std::cout << "got6" << std::endl;};
    //void handle(InNavPvt& msg){(void)msg; std::cout << "got7" << std::endl;};
    virtual void handle(InNavPosllh& msg) = 0;
    virtual void handle(InNavAttMsg& msg) = 0;
    virtual void handle(InEsfStatus& msg) = 0;
    virtual void handle(InEsfAlg& msg) = 0;
    virtual void handle(InNavTimeutc& msg) = 0;
    virtual void handle(InNavSol& msg) = 0;
    virtual void handle(InNavPvt& msg) = 0;


    common::boost_wrap::io& m_io;
    SerialPort m_serial;
    boost::asio::deadline_timer m_pollTimer;
    std::string m_device;
    boost::array<std::uint8_t, 521> m_inputBuf;
    std::vector<std::uint8_t> m_inData;
    Frame m_frame;
    unsigned int m_baudrate;
};




} // namespace simple_pos

} // namespace ublox
