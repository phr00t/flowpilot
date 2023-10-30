#include <assert.h>
#include <sched.h>
#include <iostream>
#include <cstring>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>

#include "cereal/messaging/messaging.h"
#include "selfdrive/modeld/models/driving.h"
#include "common/util.h"

#define USE_SOCKETS

using namespace std;

// function that reads data from an input stream until a given number of bytes are read
// returns true if successful, false otherwise
bool readFully(int sock, char *buffer, int length) {
    int bytesRead = 0; // number of bytes read so far
    while (bytesRead < length) { // while there are more bytes to read
        // read from the input stream and store in buffer
        int result = recv (sock, buffer + bytesRead, length - bytesRead, 0);
        if (result < 0) { // if an error occurred
	    printf("recv result: %d, errno %d", result, errno);
            return false; // return false
        }
        else if (result == 0) { // if end of stream reached
	    printf("recv result end of stream");
            return false; // return false
        }
        else { // if some bytes were read
            bytesRead += result; // update number of bytes read
        }
    }
    return true; // return true when all bytes are read
}

int getServerSocket(int port) {
    int server_sock; // server socket descriptor for receiving data from Java application
    int server_conn_sock; // server socket connection descriptor for receiving data from Java application
    struct sockaddr_in server_addr; // server address structure for receiving data from Java application
	struct sockaddr_in client_addr; // client address structure for sending data
    socklen_t sin_size; // size of sockaddr_in structure
    // set the server address and port for receiving data from Java application
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons (port); // server port for receiving data from Java application
    server_addr.sin_addr.s_addr = INADDR_ANY; // server IP address for receiving data from Java application
    memset (&(server_addr.sin_zero), 0, 8); // zero the rest of the structure
    // create a new TCP socket for receiving data from Java application
    if ((server_sock = socket (AF_INET, SOCK_STREAM, 0)) == -1) {
        perror ("socket");
        return -1;
    }

    // bind the socket to the server address and port for receiving data from Java application
    if (::bind (server_sock, (struct sockaddr *)&server_addr, sizeof (struct sockaddr)) == -1) {
        perror ("bind");
        return -1;
    }

    // listen for incoming connections on the socket for receiving data from Java application
    if (listen (server_sock, 1) == -1) {
        perror ("listen");
        return -1;
    }

    cout << "Server listening on port " << port << " for receiving data" << endl;

    // accept an incoming connection on the socket for receiving data from Java application
    sin_size = sizeof (struct sockaddr_in);
    if ((server_conn_sock = accept (server_sock, (struct sockaddr *)&client_addr, &sin_size)) == -1) {
        perror ("accept");
        return -1;
    }

    cout << "Server accepted connection from " << inet_ntoa (client_addr.sin_addr) << " for receiving data" << endl;
	
	return server_conn_sock;
}

ExitHandler do_exit;
float model_raw_preds[NET_OUTPUT_SIZE];

int main(int argc, char **argv) {

  PubMaster pm({"modelV2", "cameraOdometry"});

#ifdef USE_SOCKETS
  int server_socket = -1;
  bool success;
#else
  AlignedBuffer aligned_buf;
  std::unique_ptr<Context> context(Context::create());
  std::unique_ptr<SubSocket> subscriber(SubSocket::create(context.get(), "modelRaw"));
  assert(subscriber != NULL);
#endif

  uint32_t last_frame_id = 0;

  while (!do_exit) {
#ifdef USE_SOCKETS
	if (server_socket == -1)
		server_socket = getServerSocket(9229);
	
	// read the data from the input stream without reading the length first
	success = readFully(server_socket, (char*)&model_raw_preds[0], NET_OUTPUT_SIZE * 4);

	if (!success) { // if an error occurred or end of stream reached
		server_socket = -1;
		sched_yield();
		continue;
	}	

    model_publish(pm, 0, 0, 0, 0, model_raw_preds, 0, 0, true);
    posenet_publish(pm, 0, 0, model_raw_preds, 0, true);

#else
    // this receive should block
    std::unique_ptr<Message> msg(subscriber->receive());
    if (!msg) {
      if (errno == EINTR) {
        do_exit = true;
      }
      continue;
    }
    capnp::FlatArrayMessageReader cmsg(aligned_buf.align(msg.get()));
    cereal::Event::Reader event = cmsg.getRoot<cereal::Event>();
    auto modelRaw = event.getModelRaw();
    auto model_raw = modelRaw.getRawPredictions();

    assert(model_raw.size() == NET_OUTPUT_SIZE);
    for (int i=0; i<NET_OUTPUT_SIZE; i++)
      model_raw_preds[i] = model_raw[i];

    uint32_t vipc_dropped_frames = modelRaw.getFrameId() - last_frame_id - 1;
    
    model_publish(pm, modelRaw.getFrameId(), modelRaw.getFrameIdExtra(), modelRaw.getFrameId(), modelRaw.getFrameDropPerc()/100, 
                  model_raw_preds, modelRaw.getTimestampEof(), modelRaw.getModelExecutionTime(), modelRaw.getValid());
    posenet_publish(pm, modelRaw.getFrameId(), vipc_dropped_frames, model_raw_preds, modelRaw.getTimestampEof(), modelRaw.getValid());

    last_frame_id = modelRaw.getFrameId();
#endif

  }
  return 0;
}
