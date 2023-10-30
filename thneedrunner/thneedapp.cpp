#include <iostream>
#include <cstring>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>

#include "selfdrive/modeld/models/driving.h"
#include "thneedmodel.h"

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

// function that writes data to an output stream until a given number of bytes are written
// returns true if successful, false otherwise
bool writeFully(int sock, char *buffer, int length) {
    int bytesWritten = 0; // number of bytes written so far
    while (bytesWritten < length) { // while there are more bytes to write
        // write to the output stream from buffer
        int result = send (sock, buffer + bytesWritten, length - bytesWritten, 0);
        if (result < 0) { // if an error occurred
            return false; // return false
        }
        else if (result == 0) { // if end of stream reached
            return false; // return false
        }
        else { // if some bytes were written
            bytesWritten += result; // update number of bytes written
        }
    }
    return true; // return true when all bytes are written
}

int main () {

    int server_sock; // server socket descriptor for receiving data from Java application
    int server_conn_sock; // server socket connection descriptor for receiving data from Java application
    int client_sock; // client socket descriptor for sending data to another C++ application

    struct sockaddr_in server_addr; // server address structure for receiving data from Java application
    struct sockaddr_in client_addr; // client address structure for sending data to another C++ application

    socklen_t sin_size; // size of sockaddr_in structure

    bool success; // flag to indicate success or failure

    // create a new TCP socket for receiving data from Java application
    if ((server_sock = socket (AF_INET, SOCK_STREAM, 0)) == -1) {
        perror ("socket");
        exit (1);
    }

    // set the server address and port for receiving data from Java application
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons (8228); // server port for receiving data from Java application
    server_addr.sin_addr.s_addr = INADDR_ANY; // server IP address for receiving data from Java application
    memset (&(server_addr.sin_zero), 0, 8); // zero the rest of the structure

    // bind the socket to the server address and port for receiving data from Java application
    if (::bind (server_sock, (struct sockaddr *)&server_addr, sizeof (struct sockaddr)) == -1) {
        perror ("bind");
        exit (1);
    }

    // listen for incoming connections on the socket for receiving data from Java application
    if (listen (server_sock, 1) == -1) {
        perror ("listen");
        exit (1);
    }

    cout << "Server listening on port 8228 for receiving data from Java application" << endl;

    // accept an incoming connection on the socket for receiving data from Java application
    sin_size = sizeof (struct sockaddr_in);
    if ((server_conn_sock = accept (server_sock, (struct sockaddr *)&client_addr, &sin_size)) == -1) {
        perror ("accept");
        exit (1);
    }

    cout << "Server accepted connection from " << inet_ntoa (client_addr.sin_addr) << " for receiving data from Java application" << endl;

    // create a new TCP socket for sending data to another C++ application
    if ((client_sock = socket (AF_INET, SOCK_STREAM, 0)) == -1) {
        perror ("socket");
        exit (1);
    }

    // set the client address and port for sending data to another C++ application
    client_addr.sin_family = AF_INET;
    client_addr.sin_port = htons (9229); // client port for sending data to another C++ application
    client_addr.sin_addr.s_addr = inet_addr ("127.0.0.1"); // client IP address for sending data to another C++ application
    memset (&(client_addr.sin_zero), 0, 8); // zero the rest of the structure

    // connect to the server address and port for sending data to another C++ application
    /*if (connect (client_sock, (struct sockaddr *)&client_addr, sizeof (struct sockaddr)) == -1) {
        perror ("connect");
        exit (1);
    }*/

    cout << "Client connected to " << inet_ntoa (client_addr.sin_addr) << " on port 9229 for sending data to another C++ application" << endl;

	bool do_exit;

	// outputs
	float *model_raw_preds = new float[NET_OUTPUT_SIZE];

	// these are the inputs we need to read
	int input_imgs_len = 1572864 / 4;
	int features_len = 512 * 3072 / 4; // 1572864 with feature len 512
	int desire_len = 3200 / 4;
	int total_input_len = input_imgs_len * 2 + 1;
	float *model_input = new float[1024/4 + total_input_len + desire_len + features_len];

	int StartInput = 1024/4;
	int StartDesireInt = StartInput + total_input_len - 1;
	int StartDesire = StartInput + total_input_len;
	int StartFeatures = StartDesire + desire_len;

	// first part of model_input will be zeros
	memset(model_input, 0, 1024);

	// we will manage desire and features in here
	float *prev_desire = new float[DESIRE_LEN];

	// magic model
	ThneedModel *thneed;
	thneed = new ThneedModel("/sdcard/flowpilot/selfdrive/assets/models/f3/supercombo.thneed", model_raw_preds, NET_OUTPUT_SIZE, 0, false, NULL);

	thneed->addInput("traffic_convention", model_input, 8/4);
	thneed->addInput("nav_features", model_input, 1024/4);
	thneed->addInput("nav_instructions", model_input, 600/4);
        thneed->addInput("features_buffer", model_input + StartFeatures, features_len);
        thneed->addInput("desire", model_input + StartDesire, desire_len);

	uint32_t last_frame_id = 0;
	bool inputsSet = false;

    // loop until the connection is closed
    while (true) {

        // read the data from the input stream without reading the length first
        success = readFully(server_conn_sock, (char*)&model_input[StartInput], total_input_len * 4);

        if (!success) { // if an error occurred or end of stream reached
            break; // break from the loop
        }

        cout << "Server received data from Java application" << endl;

	int desire = *((int*)&model_input[StartDesireInt]);
        float *input_imgs = &model_input[StartInput];
        float *big_input_imgs = &model_input[StartInput + input_imgs_len]; // &model_input[1+input_imgs_len];

        // set images from android app
        if (inputsSet == false) {
            thneed->addInput("input_imgs", &model_input[StartInput], input_imgs_len);
	    thneed->addInput("big_input_imgs", &model_input[StartInput + input_imgs_len], input_imgs_len);
            inputsSet = true;
        } else {
            thneed->setInputBuffer("input_imgs", &model_input[StartInput], input_imgs_len);
	    thneed->setInputBuffer("big_input_imgs", &model_input[StartInput + input_imgs_len], input_imgs_len);
    	}

        cout << "Made inputs, now doing desire..." << endl;

	// handle desire
	float vec_desire[DESIRE_LEN] = {0};
	if (desire >= 0 && desire < DESIRE_LEN) {
	  vec_desire[desire] = 1.0;
	}

	memmove(&model_input[StartDesire], &model_input[StartDesire + DESIRE_LEN], sizeof(float) * DESIRE_LEN*HISTORY_BUFFER_LEN);
	for (int i = 1; i < DESIRE_LEN; i++) {
	  // Model decides when action is completed
	  // so desire input is just a pulse triggered on rising edge
	  if (vec_desire[i] - prev_desire[i] > .99) {
			model_input[StartDesire + DESIRE_LEN*HISTORY_BUFFER_LEN+i] = vec_desire[i];
	  } else {
			model_input[StartDesire + DESIRE_LEN*HISTORY_BUFFER_LEN+i] = 0.0;
	  }
	  prev_desire[i] = vec_desire[i];
	}

        cout << "Executing model!" << endl;

	thneed->execute();

        cout << "Handling features..." << endl;

	// handle features
	std::memmove(&model_input[StartFeatures], &model_input[StartFeatures + FEATURE_LEN], sizeof(float) * FEATURE_LEN*(HISTORY_BUFFER_LEN-1));
	std::memcpy(&model_input[StartFeatures + FEATURE_LEN*(HISTORY_BUFFER_LEN-1)], &model_raw_preds[OUTPUT_SIZE], sizeof(float) * FEATURE_LEN);

        cout << "Server generated output float array" << endl;

        // write the data to the output stream without writing the length first
        //success = writeFully(client_sock, (char*)model_raw_preds, NET_OUTPUT_SIZE);

        //if (!success) { // if an error occurred or end of stream reached
        //    break; // break from the loop
        //}

        //cout << "Client sent data to another C++ application" << endl;

        // no need to free the memory allocated for the buffer and the float array inside the loop
    }

    // free the memory allocated for the buffer and the float array only once after exiting the loop
    delete [] model_raw_preds;
    delete [] model_input;

    // close both sockets
    close (server_sock);
    close (client_sock);

    cout << "Connection closed" << endl;

    return 0;
}
