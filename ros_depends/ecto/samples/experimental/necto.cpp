#include <ecto/ecto.hpp>
#include <boost/foreach.hpp>
#include <ecto/plasm.hpp>
#include <ecto/serialization/registry.hpp>
#include <ecto/serialization/cell.hpp>

#include <boost/asio.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/tuple/tuple.hpp>
#include <iomanip>
#include <string>
#include <sstream>
#include <vector>
#include <algorithm>

namespace ecto_X
{
  using namespace ecto;
  namespace bp = boost::python;

  /// The connection class provides serialization primitives on top of a socket.
  /**
   * Each message sent using this class consists of:
   * @li An 8-byte header containing the length of the serialized data in
   * hexadecimal.
   * @li The serialized data.
   */
  class connection
  {

  public:
    typedef boost::archive::binary_oarchive oarchive;
    typedef boost::archive::binary_iarchive iarchive;

    /// Constructor.
    connection(boost::asio::io_service& io_service)
        :
          socket_(io_service)
    {
    }

    /// Get the underlying socket. Used for making a connection or for accepting
    /// an incoming connection.
    boost::asio::ip::tcp::socket&
    socket()
    {
      return socket_;
    }

    /// Asynchronously write a data structure to the socket.
    template<typename T>
    static void
    assemble_message(const T& t, std::string& header, std::string& data)
    {
      // Serialize the data first so we know how large it is.
      std::ostringstream archive_stream;
      oarchive archive(archive_stream);
      archive << t;
      data = archive_stream.str();

      // Format the header.
      std::ostringstream header_stream;
      header_stream << std::setw(header_length) << std::hex << data.size();
      header = header_stream.str();
    }

    void
    write(const std::string& header, const std::string& data, boost::system::error_code& e)
    {
      std::vector<boost::asio::const_buffer> buffers;
      buffers.push_back(boost::asio::buffer(header));
      buffers.push_back(boost::asio::buffer(data));
      boost::asio::write(socket_, buffers, boost::asio::transfer_all(), e);
    }

    template<typename T>
    void
    write(const T & t, boost::system::error_code& e)
    {
      assemble_message(t, outbound_header_, outbound_data_);
      write(outbound_header_, outbound_data_, e);
    }

    size_t
    read_header(boost::system::error_code& e)
    {
      boost::asio::read(socket_, boost::asio::buffer(inbound_header_, header_length), boost::asio::transfer_all(), e);
      if (e)
        return 0;
      size_t inbound_data_size;
      std::istringstream is(std::string(&inbound_header_[0], header_length));
      if (!(is >> std::hex >> inbound_data_size))
      {
        // Header doesn't seem to be valid. Inform the caller.
        throw std::runtime_error("Header doesn't seem to be valid.");
      }
      return inbound_data_size;
    }

    template<typename T>
    void
    read(T & t, boost::system::error_code& e)
    {
      size_t inbound_data_size = read_header(e);
      if (e)
        return;
      inbound_data_.resize(inbound_data_size);
      boost::asio::read(socket_, boost::asio::buffer(inbound_data_), boost::asio::transfer_all(), e);
      if (e)
        return;
      std::string archive_data(&inbound_data_[0], inbound_data_.size());
      std::istringstream archive_stream(archive_data);
      iarchive archive(archive_stream);
      archive >> t;
    }

  private:
    /// The underlying socket.
    boost::asio::ip::tcp::socket socket_;

    /// The size of a fixed length header.
    enum
    {
      header_length = 8
    };

    /// Holds an outbound header.
    std::string outbound_header_;
    /// Holds the outbound data.
    std::string outbound_data_;
    /// Holds an inbound header.
    char inbound_header_[header_length];
    /// Holds the inbound data.
    std::vector<char> inbound_data_;
  };

  typedef boost::shared_ptr<connection> connection_ptr;

  class client
  {
  public:
    /// Constructor starts the asynchronous connect operation.
    client(boost::asio::io_service& io_service, const std::string& host, const std::string& port)
        :
          connection_(io_service)
    {
      // Resolve the host name into an IP address.
      boost::asio::ip::tcp::resolver resolver(io_service);
      boost::asio::ip::tcp::resolver::query query(host, port);
      boost::asio::ip::tcp::resolver::iterator endpoint_iterator = resolver.resolve(query), end;
      boost::system::error_code e;
      do
      {
        boost::asio::ip::tcp::resolver::endpoint_type ep = *(endpoint_iterator++);
        std::cout << "Attempting to connect to " << ep << std::endl;
        connection_.socket().connect(ep, e);
        if (!e)
        {
          std::cout << "Connected to " << connection_.socket().remote_endpoint() << std::endl;
        }
      } while (e && endpoint_iterator != end);
      if (e)
      {
        throw std::runtime_error(e.message());
      }
    }

    void
    read(tendril& t, boost::system::error_code& e)
    {
      connection_.read(t, e);
    }

    /// The connection to the server.
    connection connection_;

    /// The data received from the server.
    tendril t_;
  };

  struct Sink
  {
    static void
    declare_params(tendrils& p)
    {
      p.declare<std::string>("url").required(true);
      p.declare<unsigned short>("port").required(true);
    }

    static void
    declare_io(const tendrils& p, tendrils& in, tendrils& out)
    {
      out.declare<tendril::none>("out");
    }

    void
    configure(const tendrils& p, const tendrils& /*in*/, const tendrils& out)
    {
      p["url"] >> url;
      p["port"] >> port;

      out_tendril = out["out"];
    }

    int
    process(const tendrils& /*in*/, const tendrils& /*out*/)
    {
      if (!client_)
      {
        client_.reset(new client(io_service, url, boost::lexical_cast<std::string>(port)));
      }
      boost::system::error_code e;
      client_->read(*out_tendril, e);
      if (e)
      {
        std::cerr << e.message() << std::endl;
        return ecto::QUIT;
      }
      return ecto::OK;
    }

    ~Sink()
    {
    }

    boost::asio::io_service io_service;
    std::string url;
    unsigned short port;
    boost::shared_ptr<client> client_;
    tendril_ptr out_tendril;
  };

  /// Serves stock quote information to any client that connects to it.
  class server
  {
  public:
    /// Constructor opens the acceptor and starts waiting for the first incoming
    /// connection.
    server(boost::asio::io_service& io_service, unsigned short port)
        :
          acceptor_(io_service, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), port))
    {
      // Start an accept operation for a new connection.
      connection_ptr new_conn(new connection(acceptor_.get_io_service()));
      acceptor_.async_accept(new_conn->socket(),
                             boost::bind(&server::handle_accept, this, boost::asio::placeholders::error, new_conn));
      std::cout << "Started server on " << acceptor_.local_endpoint() << std::endl;
    }

    /// Handle completion of a accept operation.
    void
    handle_accept(const boost::system::error_code& e, connection_ptr conn)
    {
      if (!e)
      {
        {
          boost::mutex::scoped_lock lock(mtx_);
          std::cout << "Client connected " << conn->socket().remote_endpoint() << std::endl;
          connections_.push_back(conn);
        }
        // Start an accept operation for a new connection.
        connection_ptr new_conn(new connection(acceptor_.get_io_service()));
        acceptor_.async_accept(new_conn->socket(),
                               boost::bind(&server::handle_accept, this, boost::asio::placeholders::error, new_conn));
      }
      else
      {
        std::cerr << "async_accept: " << e.message() << std::endl;
      }
    }

    void
    send_tendril(const tendril& t)
    {
      //wait for a connection before send.
      while (connections_.empty())
      {
        boost::this_thread::sleep(boost::posix_time::milliseconds(1));
      }

      connection::assemble_message(t, out_header, out_data);
      {
        boost::mutex::scoped_lock lock(mtx_); //lock so that no connections are made during this frame.
        std::vector<connection_ptr> good_connections;
        for (size_t i = 0, end = connections_.size(); i < end; i++)
        {
          boost::system::error_code e;
          connections_[i]->write(out_header, out_data, e);
          if (!e)
            good_connections.push_back(connections_[i]);
          else
            std::cerr << "Dropping client:" << e.message() << std::endl;
        }
        connections_ = good_connections;
      }
    }
  private:
    tendril data_;
    std::string out_header, out_data;
    boost::mutex mtx_;
    std::vector<connection_ptr> connections_;
    /// The acceptor object used to accept incoming socket connections.
    boost::asio::ip::tcp::acceptor acceptor_;
  }
  ;

  struct Source
  {
    static void
    declare_params(tendrils& p)
    {
      p.declare<unsigned short>("port").required(true);
    }

    static void
    declare_io(const tendrils& /*p*/, tendrils& in, tendrils& /*out*/)
    {
      in.declare<tendril::none>("in");
    }

    void
    configure(const tendrils& p, const tendrils& in, const tendrils& /*out*/)
    {
      p["port"] >> port;
      in_tendril = in["in"];
    }

    int
    process(const tendrils& /*in*/, const tendrils& /*out*/)
    {
      if (!io_service)
      {
        io_service.reset(new boost::asio::io_service);
        server_.reset(new server(*io_service, port));
        runner.reset(new boost::thread(boost::bind(&boost::asio::io_service::run, io_service)));
      }
      server_->send_tendril(*in_tendril);
      return ecto::OK;
    }

    unsigned short port;
    tendril_ptr in_tendril;
    boost::shared_ptr<boost::asio::io_service> io_service;
    boost::shared_ptr<server> server_;
    boost::shared_ptr<boost::thread> runner;
  };
}
ECTO_CELL(ecto_X, ecto_X::Sink, "Sink", "Subscribes to tendril over the network.");
ECTO_CELL(ecto_X, ecto_X::Source, "Source", "Publishes a tendril over the network.");
