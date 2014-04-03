
#include <ctype.h>

#include <stdint.h>

#include <iostream>
#include <iomanip>
#include <vector>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

class nmea_port {
public:

    static const size_t max_sentence_length = 1024;

    nmea_port(boost::asio::io_service& io_service,
              std::string& port_name,
              unsigned int baud_rate)
        : m_serial(io_service),
          m_buffer(max_sentence_length)
    {
        m_serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
        m_serial.open(port_name);

        read_next();
    }

    virtual ~nmea_port() { }

private:

    boost::asio::serial_port m_serial;
    boost::asio::streambuf   m_buffer;

    void read_next() {
        boost::asio::async_read_until
            (m_serial, m_buffer, std::string("\r\n"),
             boost::bind(&nmea_port::handle_line, this,
                         boost::asio::placeholders::error,
                         boost::asio::placeholders::bytes_transferred));
    }

    void handle_line(const boost::system::error_code& error, size_t bytes_received) {
        if(!error) {
            boost::asio::streambuf::const_buffers_type bufs = m_buffer.data();
            std::string line(boost::asio::buffers_begin(bufs),
                             boost::asio::buffers_begin(bufs) + bytes_received);

            std::cerr << "Sentence: " << line << std::endl;

            m_buffer.consume(bytes_received);

            read_next();
        } else {
            std::cerr << "Error: " << error << std::endl;
        }
    }

};

class garmin_sample;
typedef boost::shared_ptr<garmin_sample> garmin_sample_ptr;



class garmin_sample {
public:
    garmin_sample(uint16_t angle, double range)
        : m_angle(angle), m_range(range)
    { }

    ~garmin_sample() { }

    void set_samples(size_t n, const uint8_t* samples) {
        m_samples.assign((uint8_t*)samples, (uint8_t*)(samples + n));
    }

private:
    uint16_t             m_angle;
    double               m_range;
    std::vector<uint8_t> m_samples;
};

class garmin_radar {
public:

    static const int max_frame_size = 1500;
    static const int multicast_port = 50100;
    static const int control_port   = 50101;

    garmin_radar(boost::asio::io_service& io_service,
                 const boost::asio::ip::address& local_address,
                 const boost::asio::ip::address& remote_address,
                 const boost::asio::ip::address& multicast_address)
        : m_control_socket(io_service),
          m_multicast_socket(io_service)
    {
        // set up control socket
        boost::asio::ip::udp::endpoint local_control_endpoint(local_address, control_port);
        boost::asio::ip::udp::endpoint remote_control_endpoint(remote_address, control_port);
        m_control_socket.open(local_control_endpoint.protocol());
        m_control_socket.set_option(boost::asio::ip::udp::socket::reuse_address(true));
        m_control_socket.bind(local_control_endpoint);
        m_control_socket.connect(remote_control_endpoint);

        // set up multicast socket
        boost::asio::ip::udp::endpoint multicast_endpoint(local_address, multicast_port);
        m_multicast_socket.open(multicast_endpoint.protocol());
        m_multicast_socket.set_option(boost::asio::ip::udp::socket::reuse_address(true));
        m_multicast_socket.bind(multicast_endpoint);
        m_multicast_socket.set_option(boost::asio::ip::multicast::join_group(multicast_address));

        // start receiving
        receive_next();
    }

    virtual ~garmin_radar() { }

    void power_off() {
        std::cout << "Powering off the radar" << std::endl;
        send_control_ushort(frame_type_power, 1);
    }

    void power_on() {
        std::cout << "Powering on the radar" << std::endl;
        send_control_ushort(frame_type_power, 2);
    }

    void set_ftc(bool on) {
        std::cout << "Setting FTC to " << on << std::endl;
        send_control_uchar(frame_type_ftc, on ? 1 : 0);
    }

    void set_crosstalk(bool on) {
        std::cout << "Setting CROSSTALK to " << on << std::endl;
        send_control_uchar(frame_type_crosstalk, on ? 1 : 0);
    }

    void set_gain(bool manual, unsigned int value = 0) {
        std::cout << "Setting GAIN to ";
        if(manual) {
            std::cout << value << std::endl;
        } else {
            std::cout << "AUTO" << std::endl;
            value = 344; // XXX
        }
        send_control_uint(frame_type_gain, value);
    }

    void set_range(double range_nm) {
        double range_m = range_nm * 1852.0;
        unsigned short range = range_m - 1;
        std::cout << "Setting range to " << range_nm << " nm (" << range_m << " m, value " << range << ")" << std::endl;
        send_control_uint(frame_type_range, range);
    }

private:

    // UDP socket for sending control commands
    boost::asio::ip::udp::socket   m_control_socket;
    // UDP socket for receiving multicast data
    boost::asio::ip::udp::socket   m_multicast_socket;

    // Buffer for sending endpoint of frame
    boost::asio::ip::udp::endpoint m_frame_sender;
    // Buffer for frame data
    char                           m_frame_data[max_frame_size];

    // The various frame types we know about
    static const unsigned int frame_type_power       = 0x2b2; /* power control */
    static const unsigned int frame_type_range       = 0x2b3; /* range control */
    static const unsigned int frame_type_gain        = 0x2b4; /* gain control */
    static const unsigned int frame_type_seaclutter  = 0x2b5; /* sea clutter control */
    static const unsigned int frame_type_rainclutter = 0x2b6; /* rain clutter control */
    static const unsigned int frame_type_offset      = 0x2b7; /* dome mounting offset */
    static const unsigned int frame_type_ftc         = 0x2b8; /* fast time constant control */
    static const unsigned int frame_type_crosstalk   = 0x2b9; /* crosstalk control */

    static const unsigned int frame_type_scanline    = 0x2a3;
    static const unsigned int frame_type_status      = 0x2a5;
    static const unsigned int frame_type_response    = 0x2a7;
    static const unsigned int frame_type_unknown1    = 0x2ac;

    struct frame_common {
        unsigned int frame_type;
    } __attribute__((packed));

    struct frame_control_uint {
        unsigned int frame_type;
        unsigned int len1;
        unsigned int data;
    } __attribute__((packed));
    struct frame_control_ushort {
        unsigned int   frame_type;
        unsigned int   len1;
        unsigned short data;
    } __attribute__((packed));
    struct frame_control_uchar {
        unsigned int  frame_type;
        unsigned int  len1;
        unsigned char data;
    } __attribute__((packed));

    struct frame_scanline {
        unsigned int   frame_type;
        unsigned int   len1;
        unsigned short angle;
        unsigned short scan_length_bytes;
        unsigned int   display_meters;
        unsigned int   range_meters;
        unsigned char  gain_level[4];
        unsigned char  sea_clutter[4];
        unsigned char  rain_clutter[4];
        short          dome_offset;
        unsigned char  ftc;
        unsigned char  crosstalk;
        unsigned char  _pad0[4];
        unsigned char  timed_transmit[4];
        unsigned char  dome_speed;
        unsigned char  _pad1[7];
        unsigned char  line_data[];
    } __attribute__((packed));

    struct frame_status {
        unsigned int   frame_type;
        unsigned int   len1;
        unsigned short state;
        unsigned short countdown;
        unsigned int   unknown1;
        unsigned int   unknown2;
    } __attribute__((packed));

    struct frame_response {
        unsigned int   frame_type;
        unsigned int   len1;
        unsigned char  _pad0[4];
        unsigned int   range_meters;
        unsigned char  gain_level[4];
        unsigned char  sea_clutter[4];
        unsigned char  rain_clutter[4];
        short          dome_offset;
        unsigned char  ftc;
        unsigned char  crosstalk;
        unsigned char  _pad1[4];
        unsigned char  timed_transmit[4];
        unsigned char  dome_speed;
        unsigned char  _pad2[7];
    } __attribute__((packed));

    void send_control_uint(unsigned int type, unsigned int data) {
        frame_control_uint frame;
        frame.frame_type = type;
        frame.len1 = 4;
        frame.data = data;
        m_control_socket.send(boost::asio::buffer(&frame, sizeof(frame)));
    }

    void send_control_ushort(unsigned int type, unsigned short data) {
        frame_control_ushort frame;
        frame.frame_type = type;
        frame.len1 = 2;
        frame.data = data;
        m_control_socket.send(boost::asio::buffer(&frame, sizeof(frame)));
    }

    void send_control_uchar(unsigned int type, unsigned char data) {
        frame_control_uchar frame;
        frame.frame_type = type;
        frame.len1 = 1;
        frame.data = data;
        m_control_socket.send(boost::asio::buffer(&frame, sizeof(frame)));
    }

    void receive_next() {
        m_multicast_socket.async_receive_from
            (boost::asio::buffer(m_frame_data, sizeof(m_frame_data)), m_frame_sender,
             boost::bind(&garmin_radar::handle_frame, this,
                         boost::asio::placeholders::error,
                         boost::asio::placeholders::bytes_transferred));
    }

    void handle_frame(const boost::system::error_code& error, size_t bytes_received) {
        if(!error) {
            if(bytes_received < sizeof(struct frame_common)) {
                std::cerr << "Short frame (length " << bytes_received << ")" << std::endl;
                return;
            }
            const frame_common *frame = (const frame_common *)m_frame_data;
            switch(frame->frame_type) {
            case frame_type_status:
                if(bytes_received == sizeof(struct frame_status)) {
                    handle_status_frame((const struct frame_status *)m_frame_data);
                } else {
                    std::cerr << "Status frame with bad length " << bytes_received << ", expected " << sizeof(frame_status) << std::endl;
                }
                break;
            case frame_type_scanline:
                if(bytes_received >= sizeof(struct frame_scanline)) {
                    handle_scanline_frame((const struct frame_scanline *)m_frame_data);
                } else {
                    std::cerr << "Scanline frame with bad length " << bytes_received << ", expected >= " << sizeof(frame_scanline) << std::endl;
                }
                break;
            case frame_type_response:
                if(bytes_received == sizeof(struct frame_response)) {
                    handle_response_frame((const struct frame_response *)m_frame_data);
                } else {
                    std::cerr << "Response frame with bad length " << bytes_received << ", expected " << sizeof(frame_response) << std::endl;
                }
                break;
            case frame_type_unknown1:
                break;
            default:
                std::cerr << "Unknown frame type " << frame->frame_type << std::endl;
            }

            receive_next();
        } else {
            std::cerr << "Error: " << error << std::endl;
        }
    }

    void handle_status_frame(const struct frame_status *status) {
        switch(status->state) {
        case 1:
            std::cerr << "Warming up, ready in " << status->countdown << std::endl;
            break;
        case 3:
            std::cerr << "Standby" << std::endl;
            break;
        case 4:
            std::cerr << "Active" << std::endl;
            break;
        case 5:
            std::cerr << "Spinup" << std::endl;
            break;
        default:
            std::cerr << "Unknown state " << status->state << std::endl;
            break;
        }
    }

    void handle_scanline_frame(const struct frame_scanline *scanline) {
        std::cerr << "Scanline: angle " << scanline->angle << " range " << scanline->range_meters << " length " << scanline->scan_length_bytes << std::endl;

        std::vector<garmin_sample_ptr> samples;
        const unsigned char *data = scanline->line_data;
        unsigned short n = scanline->scan_length_bytes / 4;
        unsigned short i, k;
        for(i = 0; i < 4; i++) {
            double angle = scanline->angle + i * 0.5;

            std::cout << "angle " << std::fixed << std::setprecision(1) << angle << ": ";
            for(k = 0; k < n; k++) {
                unsigned short p = k + i * n;
                std::cout << ((int)data[p]) << " ";
            }
            std::cout << std::endl;

            garmin_sample_ptr sample =
                boost::make_shared<garmin_sample>
                  ((scanline->angle * 100) + (i * 25),
                   scanline->range_meters);
            sample->set_samples(n, data);
            samples.push_back(sample);
        }

        print_controls(scanline);
    }

    void handle_response_frame(const struct frame_response *frame) {
        print_controls(frame);
    }

    template <class T>
    void print_controls(T *frame) {
        std::cerr << "Range " << frame->range_meters << " m" << std::endl;

        int gain_mode = frame->gain_level[1];
        int gain_level = frame->gain_level[0];
        std::cerr << "Gain ";
        if(gain_mode) {
            std::cerr << "AUTO(" << gain_level << "%)";
        } else {
            std::cerr << gain_level << "%";
        }
        std::cerr << ", ";
        std::cerr << "FTC " << (frame->ftc?"ON":"OFF");
        std::cerr << ", ";
        std::cerr << "Crosstalk " << (frame->crosstalk?"ON":"OFF");
        std::cerr << std::endl;

        //std::cerr << "SCltrLvl " << (int)scanline->sea_clutter[0] << std::endl;
        //std::cerr << "SCltrMode " << (int)scanline->sea_clutter[1] << std::endl;
        //std::cerr << "RCltrLvl " << (int)scanline->rain_clutter[0] << std::endl;
        //std::cerr << "DomeOffset " << scanline->dome_offset << std::endl;
    }

};


class commander {
    
public:
    commander(boost::asio::io_service& ios, garmin_radar* radar)
        : m_cmd_input(ios, ::dup(STDIN_FILENO)),
          m_radar(radar)
    {
        read_cmd();
    }

    void read_cmd() {
        boost::asio::async_read
            (m_cmd_input,
             boost::asio::buffer(&m_cmd_buf, sizeof(m_cmd_buf)),
             boost::bind(&commander::handle_cmd, this,
                         boost::asio::placeholders::error));
    }
    
    void handle_cmd(const boost::system::error_code& error) {
        if(!error) {
            if(isgraph(m_cmd_buf)) {
                std::cout << "Command " << m_cmd_buf << std::endl;
                switch(m_cmd_buf) {
                case 'a':
                    m_radar->power_on();
                    break;
                case 'q':
                    m_radar->power_off();
                    break;
                case 'w':
                    m_radar->set_range(0.25);
                    break;
                case 'e':
                    m_radar->set_range(0.50);
                    break;
                case 'r':
                    m_radar->set_range(1.00);
                    break;
                case 't':
                    m_radar->set_range(3.00);
                    break;
                case 'x':
                    m_radar->set_crosstalk(false);
                    break;
                case 'X':
                    m_radar->set_crosstalk(true);
                    break;
                case 'c':
                    m_radar->set_ftc(false);
                    break;
                case 'C':
                    m_radar->set_ftc(true);
                    break;
                case 's':
                    m_radar->set_gain(false);
                    break;
                case 'd':
                    m_radar->set_gain(true, 0);
                    break;
                case 'f':
                    m_radar->set_gain(true, 25);
                    break;
                case 'g':
                    m_radar->set_gain(true, 50);
                    break;
                case 'h':
                    m_radar->set_gain(true, 75);
                    break;
                case 'j':
                    m_radar->set_gain(true, 100);
                    break;
                default:
                    break;
                }
            }
            read_cmd();
        }
    }

    boost::asio::posix::stream_descriptor m_cmd_input;
    char                                  m_cmd_buf;

    garmin_radar *m_radar;
};

int
main(int argc, char **argv) {
    try {
        boost::asio::io_service ios;
        garmin_radar radar(ios,
             boost::asio::ip::address::from_string("0.0.0.0"),
             boost::asio::ip::address::from_string("172.16.2.0"),
             boost::asio::ip::address::from_string("239.254.2.0"));
        commander cmd(ios, &radar);

        ios.run();
        
    } catch (std::exception& e) {
        std::cerr << "Exception: " << e.what() << "\n";
    }

    return 0;
}
