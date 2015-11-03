#ifndef CTCPSOCKET_H_
#define CTCPSOCKET_H_

#include <vector>
#include <string>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>


namespace cola2
{
	namespace io
	{
		typedef std::vector< unsigned char > DataBuffer ;
		
		class TcpSocket 
		{
			private:
				boost::asio::io_service io_service_;
				boost::asio::ip::tcp::socket socket_;
				
			public:
				TcpSocket() :
					io_service_(),
					socket_( io_service_ )
				{}
				
				~TcpSocket(){}
				
				void
				connect( const std::string& IP,
						const unsigned short port )
				{
					boost::asio::ip::tcp::resolver resolver( io_service_ ) ;
					boost::asio::ip::tcp::resolver::query query( IP, boost::lexical_cast< std::string >( port ) ) ;
					boost::asio::ip::tcp::resolver::iterator endpoint_iterator = resolver.resolve( query ) ;
					boost::asio::ip::tcp::resolver::iterator end ; // End marker.n.

					boost::system::error_code error = boost::asio::error::host_not_found ;

					while ( error && endpoint_iterator != end ) {
						 socket_.close() ;
						 socket_.connect( *endpoint_iterator++ , error ) ;
					}

					if ( error )
						 throw boost::system::system_error( error ) ;

				}

				unsigned char
				readByte( const unsigned int msTimeout )
					throw ( std::runtime_error )
				{
					DataBuffer dataBuffer ;
					read( dataBuffer, 1, msTimeout ) ;

					return *( dataBuffer.data() );
				}



				void
				read (DataBuffer& dataBuffer,
					  const unsigned int numOfBytes,
					  const unsigned int msTimeout )
					throw ( std::runtime_error )
				{
					unsigned int readBytes;

					if ( numOfBytes == 0 ) {
						readBytes = socket_.available() ;
					}
					else {
						readBytes = numOfBytes ;
					}


					dataBuffer.resize( readBytes ) ;

					if ( msTimeout == 0 ) {
						// Perform synchronous read
						boost::system::error_code ec ;

						size_t numOfBytesRead = boost::asio::read( socket_, boost::asio::buffer( dataBuffer ), boost::asio::transfer_all(), ec ) ;

						if ( numOfBytesRead != readBytes )
							throw std::runtime_error( "Unexpected number of bytes read" ) ;


						if ( ec )
							throw std::runtime_error( "Could not read from TCP socket : " + ec.message() ) ;

					}
					else {
						// Perform asynchronous timed read

						boost::optional< boost::system::error_code > timerResult ;

						boost::asio::deadline_timer timer( socket_.get_io_service() ) ;
						timer.expires_from_now( boost::posix_time::milliseconds( msTimeout ) ) ;
						timer.async_wait( boost::bind( &TcpSocket::timedReadHandler, this, &timerResult, _1 ) ) ;

						boost::optional< boost::system::error_code > readResult ;
						boost::asio::async_read( socket_,
												 boost::asio::buffer( dataBuffer ),
												 boost::bind( &TcpSocket::timedReadHandler, this, &readResult, _1 ) ) ;

						socket_.get_io_service().reset() ;

						boost::system::error_code ec ;
						bool timeoutOccurred = false ;

						while ( socket_.get_io_service().run_one( ec ) ) {
							if ( readResult ) {
								timer.cancel( ec ) ;
							}
							else if ( timerResult ) {
								socket_.cancel( ec ) ;
								timeoutOccurred = true ;
							}
						}
						
						if ( timeoutOccurred )
							throw std::runtime_error( "TimeOut): " + readResult->message() ) ;

						if ( *readResult )
							throw std::runtime_error( "Could not perform timed read from TCP socket (1): " + readResult->message() ) ;

						if ( ec )
							throw std::runtime_error( "Could not perform timed read from TCP socket (2): " + ec.message() ) ;
					}
				}



				void
				readUntil( DataBuffer& dataBuffer,
						   	   	   	   const unsigned char delimiter,
						   	   	   	   const unsigned int msTimeout )
					throw ( std::runtime_error )
				{
					std::string line = readLine( msTimeout, delimiter ) ;
					dataBuffer.clear() ;
					dataBuffer.resize( line.length() ) ;
					std::copy( line.begin(), line.end(), dataBuffer.begin() ) ;
				}



				std::string
				readLine ( const unsigned int msTimeout,
							   	   	   const unsigned char lineTerminator )
					throw ( std::runtime_error )
				{
					std::string result ;
					char byte = 0 ;

					if ( msTimeout == 0 ) {
						// Perform synchronous read
						boost::system::error_code ec ;

						do {
							size_t numOfBytesRead = boost::asio::read( socket_, boost::asio::buffer( &byte, 1 ), boost::asio::transfer_all(), ec ) ;

							if ( ec )
								throw std::runtime_error( "Could not read from TCP socket : " + ec.message() ) ;

							if ( numOfBytesRead != 1 )
								throw std::runtime_error( "Unexpected number of bytes read" ) ;

							result += byte ;
						}
						while ( byte != lineTerminator ) ;


					}
					else {
						// Perform asynchronous timed read

						boost::optional< boost::system::error_code > timerResult ;

						boost::asio::deadline_timer timer( socket_.get_io_service() ) ;
						timer.expires_from_now( boost::posix_time::milliseconds( msTimeout ) ) ;
						timer.async_wait( boost::bind( &TcpSocket::timedReadHandler, this, &timerResult, _1 ) ) ;

						boost::optional< boost::system::error_code > readResult ;
						boost::asio::async_read( socket_,
												 boost::asio::buffer( &byte, 1 ),
												 boost::bind( &TcpSocket::timedReadHandler, this, &readResult, _1 ) ) ;

						socket_.get_io_service().reset() ;

						boost::system::error_code ec ;
						bool timeoutOccurred = false ;
						bool readEnd = false ;

						while ( socket_.get_io_service().run_one( ec )  ) {
							if ( readResult ) {

								if ( !readEnd ) {

									result += byte ;

									if ( byte != lineTerminator ) {
										readResult.reset() ;
										boost::asio::async_read( socket_,
															 	 boost::asio::buffer( &byte , 1 ),
															 	 boost::bind( &TcpSocket::timedReadHandler, this, &readResult, _1 ) ) ;
									}
									else {
										//El while encara torna a entrar una altra vegada
										//un cop fet el timer.cancel( eo ) . Aixo no passa amb
										//socket_.cancel( eo ) ja que es para en la següent execució.
										timer.cancel( ec ) ;
										readEnd = true ;
									}
								}
							}
							else if ( timerResult ) {
								socket_.cancel( ec ) ;
								timeoutOccurred = true ;
							}
						}

						if ( timeoutOccurred )
							throw std::runtime_error( "TimeOut): " + readResult->message() ) ;


						if ( *readResult )
							throw std::runtime_error( "Could not perform timed read from TCP socket (1): " + readResult->message() ) ;

						if ( ec )
							throw std::runtime_error( "Could not perform timed read from TCP socket (2): " + ec.message() ) ;
					}

					return result;
				}

				void
				writeByte ( const unsigned char dataByte )
					throw ( std::runtime_error )
				{
					DataBuffer dataBuffer ;

					dataBuffer.resize( 1 ) ;
					*( dataBuffer.data() ) = dataByte ;

					write( dataBuffer ) ;
				}

				void
				write ( const DataBuffer& dataBuffer )
					throw ( std::runtime_error )
				{
					boost::system::error_code ec ;
					size_t numOfBytesWrite = boost::asio::write( socket_, boost::asio::buffer( dataBuffer ), boost::asio::transfer_all(), ec ) ;

					if ( ec )
						throw std::runtime_error( "Could not write to TCP socket: " + ec.message() ) ;

					if (numOfBytesWrite != dataBuffer.size() )
						throw std::runtime_error( "Unexpected number of bytes sent" ) ;
				}



				void
				write ( const std::string& dataString )
					throw ( std::runtime_error )
				{
					boost::system::error_code ec ;
					size_t numOfBytesWrite = boost::asio::write( socket_, boost::asio::buffer( dataString.data(), dataString.length() ), boost::asio::transfer_all(), ec ) ;

					if ( ec )
						throw std::runtime_error( "Could not write to TCP socket: " + ec.message() ) ;

					if (numOfBytesWrite !=  dataString.length() )
						throw std::runtime_error( "Unexpected number of bytes sent" ) ;
				}


				void
				close()
				{
					boost::system::error_code ec ;

					socket_.close( ec ) ;

					if ( ec )
						 throw boost::system::system_error( ec ) ;
				}



				void
				timedReadHandler( boost::optional< boost::system::error_code >* a,
											       const boost::system::error_code& b )
				{
					a->reset( b ) ;
				}
		};
	};
};
				
#endif /* CTCPSOCKET_H_ */

