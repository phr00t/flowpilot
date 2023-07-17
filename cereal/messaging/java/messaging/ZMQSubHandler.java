package messaging;
//zmq

import ai.flow.definitions.Definitions;
import org.capnproto.MessageReader;
import org.capnproto.Serialize;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.zeromq.ZMQ;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class ZMQSubHandler{
    public static final ZMQ.Context context = Factory.getContext();
    public static final PortMap portMap = Factory.getPortmap();
    public static final Logger logger = LoggerFactory.getLogger(ZMQSubHandler.class);
    public int subCount = 0;
    public Map<String, Integer> pollTopics = new HashMap<>();
    public ZMQ.Poller poller = context.poller();
    public Map<String, ZMQ.Socket> sockets = new HashMap<>();
    public final boolean conflate;

    public ZMQSubHandler(boolean conflate){
        this.conflate = conflate;
    }

    public boolean createSubscribers(List<String> topicList){
        boolean status = true;
        for (String topic : topicList){
            status = status & createSubscriber(topic);
        }
        return status;
    }

    public boolean createSubscriber(String topic){
        ZMQ.Socket socket;
        int port;
        port = ZMQSubHandler.portMap.services.get(topic).port;
        socket = context.socket(ZMQ.SUB);
        socket.setConflate(ZMQSubHandler.portMap.services.get(topic).keepLast);
        socket.connect(Utils.getSocketPath(Integer.toString(port)));
        socket.subscribe("".getBytes());
        poller.register(socket, ZMQ.Poller.POLLIN);
        this.sockets.put(topic, socket);
        this.pollTopics.put(topic, subCount);
        logger.debug("Subscriber created: {}", topic);
        subCount++;
        return true;
    }

    public void releaseSubscribers(List<String> topicList){
        for (String topic : topicList){
            this.sockets.get(topic).close();
        }
    }

    public void releaseAll(){
        for(String topic : this.sockets.keySet()) {
            this.sockets.get(topic).close();
        }
    }

    public byte[] recvMultipart(String topic){
        ZMQ.Socket socket;
        socket = this.sockets.get(topic);
        socket.recv(); // receive topic
        return socket.recv(); // actual data
    }

    public ByteBuffer recvBuffer(String topic){
        ZMQ.Socket socket = this.sockets.get(topic);
        return ByteBuffer.wrap(socket.recv());
    }

    public Definitions.Event.Reader recv(String topic){
        try {
            ByteBuffer buffer = recvBuffer(topic);
            MessageReader messageReader = Serialize.read(buffer);
            buffer.rewind();
            return messageReader.getRoot(Definitions.Event.factory);
        } catch (IOException e) {
            e.printStackTrace();
            return null;
        }
    }

    public boolean updated(String topic){
        poller.poll(0);
        return poller.pollin(pollTopics.get(topic));
    }

    public Map<String, Boolean> updated(List<String> topicList){
        poller.poll(0);
        Map<String, Boolean> status = new HashMap<>();
        for (String topic : topicList){
            status.put(topic, poller.pollin(pollTopics.get(topic)));
        }
        return status;
    }

    public Map<String, byte[]> getData(List<String> topicList){
        Map<String, byte[]> allData = new HashMap<>();
        byte[] data;
        for (String topic : topicList){
            data = getData(topic);
            allData.put(topic, data);
        }
        return allData;
    }

    public byte[] getData(String topic){
        ZMQ.Socket socket = this.sockets.get(topic);
        return socket.recv();
    }
}
