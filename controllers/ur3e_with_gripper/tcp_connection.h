#pragma once

#pragma comment (lib, "Ws2_32.lib")

#include <winsock2.h>
#include <windows.h>
#include <ws2tcpip.h>
#include <stdlib.h>
#include <stdio.h>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <string>
#include <thread>
#include <bitset>


#include "../Proto/tcp_data_message.pb.h"

namespace tcp_io_device {
  /**
  * SafeQueue is a thread-safe queue used to pass data from the TcpIoDevice to the TCPConnection for outgoing
  * and the other way around for incoming messages.
  */
  class SafeQueue
  {
  public:
    /**
    * Constructor with the name of the queue. Defaults the max number of messages in the queue to 1.
    */
    SafeQueue()
      : queue_()
      , mutex_()
    {
      max_elements_ = 1;
    }

    /**
    * Constructor with the name of the queue. Sets the max number of messages in the queue accordingly.
    * \param max_elements The max number of messages in the queue before old messages are deleted if a new one is enqueued.
    */
    SafeQueue(int max_elements)
      : queue_()
      , mutex_()
    {
      max_elements_ = max_elements;
    }

    ~SafeQueue()
    {}

    /**
    * Adds a new element to the queue, also deletes old messages, if the number of messages in the queue is >= max_elements.
    * \param t The message to enqueue.
    */
    void enqueue(std::unique_ptr<TCPMessage> t)
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      while (queue_.size() >= max_elements_) {
        if (queue_.front()->messagetype() == TCPMessage_Type_DATA) {
          queue_.front()->release_datamessage();
        }
        queue_.pop();
      }
      queue_.push(std::move(t));
    }


    /**
    * Returns the front element (oldest) of the queue and deletes it.
    * \return Oldest message in the queue.
    */
    std::unique_ptr<TCPMessage> dequeue()
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      if (queue_.empty()) {
        return NULL;
      }
      std::unique_ptr<TCPMessage> val = std::move(queue_.front());
      queue_.pop();
      return val;
    }

    /**
    * Clears all entries of the queue.
    */
    void clear() {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      while (queue_.size() > 0) {
        if (queue_.front()->messagetype() == TCPMessage_Type_DATA) {
          queue_.front()->release_datamessage();
        }
        queue_.pop();
      }
    }



  private:
    std::queue<std::unique_ptr<TCPMessage>> queue_;
    mutable std::recursive_mutex mutex_;
    int max_elements_;
  };



  class TCPConnection {

  public:

    static std::map<int, std::string> type_to_name_map_;

    /**
    * Constructor for the TCPConnection used in a seperate thread to communicate with the environment simulation
    * \param receive_queue The queue used to pass incoming messages to the TcpIoDevice.
    * \param send_queue The queue used to pass outgoing messages from the TcpIoDevice.
    * \param msg_length_buf_size The number of bytes used to store the message length of the serialized protobuf message (should be 8)
    */
    TCPConnection(std::shared_ptr<SafeQueue> receive_queue, std::shared_ptr<SafeQueue> send_queue, uint64_t msg_length_buf_size);
    ~TCPConnection();

    /**
    * Opens a socket to connect to a client on the passed port.
    * \param port The port used to communicate with the client.
    */
    int establishConnection(std::string host, std::string port);

    /**
    * Starts the communication between environment simulation (client) and the TCPConnection (server).
    */
    void start();

    /**
    * Stops the communication
    */
    void stop();

    /**
    * Returns true if the TCPConnection is running.
    * \return true if running, false otherwise.
    */
    bool isRunning() { return state_ == RUNNING; }

    /**
    * Check the socket if there is incoming data ready. This does not block.
    * \param fd The socket file descriptor.
    * \return 1 if data is ready, 0 if no data is ready, -1 for error.
    */
    static int receiveIsReady(SOCKET fd);

  protected:

    typedef enum {
      NOT_STARTED = 0,
      RUNNING = 1,
      STOPPED = 2,
    }State;

    State state_;


    std::shared_ptr<std::thread> tcp_background_thread_;

    SOCKET tcp_socket_;

    uint64_t msg_buf_size_;
    uint64_t msg_length_buf_size_;

    std::shared_ptr<SafeQueue> incoming_queue_;
    std::shared_ptr<SafeQueue> outgoing_queue_;

    /**
    * Handles the TCP connection in the background by checking for new outgoing and incoming messages, dequeueing and enqueueing the
    * SafeQueues, respectively. Repeatedly checks for new messages on the socket and parses them to TCPMessage objects. Takes TCPMessage
    * objects from the outgoing_queue_ and parses them into a byte-stream to send to the client.
    */
    void tcpBackgroundHandler();

    /**
    * Method to receive data from the socket and parse them into a TCPMessage
    * \return A unique_ptr to a TCPMessage for enqueuing it into the receive_queue_.
    */
    std::unique_ptr<TCPMessage> receiveMessage();

    /**
    * Converts a message to a byte-stream and sends it to the client.
    * \param msg The TCPMessage to send.
    * \return The number of bytes sent. If <= 0 an error occured while sending the message.
    */
    int sendMessage(std::unique_ptr<TCPMessage> msg);
  };
} // namespace tcp_io_device