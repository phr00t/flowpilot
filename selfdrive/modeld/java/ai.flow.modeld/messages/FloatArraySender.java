package ai.flow.modeld.messages;

import java.io.*;
import java.net.*;
import java.nio.*;

public class FloatArraySender {

    private Socket socket; // the socket to communicate with the C++ application
    private DataOutputStream out; // the output stream to send data through the socket

    // constructor that takes the IP address and port number of the C++ application as parameters
    public FloatArraySender (String address, int port) {
        try {
            // create a new socket with the given address and port
            socket = new Socket (address, port);
            // get the output stream of the socket
            out = new DataOutputStream (socket.getOutputStream ());
        } catch (Exception e) {}
    }

    // method that takes a float array as parameter and sends it to the C++ application
    public void sendInputsOut(byte[] bytes, int desire) {
        try {
            // write the byte array to the output stream
            out.writeInt(desire);
            out.write(bytes);
            // flush the output stream
            out.flush ();
        } catch (Exception e) {}
    }

    // method that closes the socket and the output stream
    public void close () {
        try {
            // close the output stream
            out.close ();
            // close the socket
            socket.close ();
        } catch (Exception e) {}
    }
}
