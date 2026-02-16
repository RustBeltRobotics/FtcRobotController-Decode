package org.firstinspires.ftc.teamcode;

import java.io.DataOutputStream;
import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;
import java.nio.ByteBuffer;
import java.nio.charset.StandardCharsets;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.atomic.AtomicInteger;

public class WebTelemetryStreamer implements Runnable {

    ServerSocket serverSocket;

    Socket currentClient;

    DataOutputStream currentClientOut;

    ConcurrentLinkedQueue<byte[]> messageQueue;

    int port = 8082;

    // Channel tracking
    private final ConcurrentHashMap<String, Integer> channelMap = new ConcurrentHashMap<>();
    private final AtomicInteger nextChannelId = new AtomicInteger(0);
    private volatile boolean channelMapSent = false;

    @Override
    public void run() {
        try {
            System.out.print("Web telemetry streamer listening on port ");
            System.out.println(this.port);
            listen(this.port);
        } catch (IOException e) {
            System.out.print("Uh oh:");
            System.out.println(e.getMessage());
        }
    }

    WebTelemetryStreamer(int port) {
        this.port = port;
        this.messageQueue = new ConcurrentLinkedQueue<>();
    }

    private void waitForClient() throws IOException {
        this.currentClient = this.serverSocket.accept();

        String response_start =
                "HTTP/1.1 200 OK\r\n"+
                        "Date: Tue, 23 Apr 2024 10:30:00 GMT\r\n"+
                        "Content-Type: application/octet-stream\r\n"+
                        "Cache-Control: no-cache\r\n"+
                        "Connection: keep-alive\r\n"+
                        "Access-Control-Allow-Origin: *\r\n"+
                        "\r\n";

        this.currentClientOut = new DataOutputStream(currentClient.getOutputStream());
        currentClientOut.write(response_start.getBytes(StandardCharsets.UTF_8));
        currentClientOut.flush();

        // Send channel map
        sendChannelMap();
        channelMapSent = true;

        System.out.println("Web telemetry streamer: Client connected");
    }

    public void stop() throws IOException {
        this.serverSocket.close();
    }

    private void listen(int port) throws IOException {
        this.serverSocket = new ServerSocket(port);

        while (true) {
            waitForClient();
            messageQueue.clear();

            while (currentClient.isConnected()) {
                try {
                    byte[] message = messageQueue.poll();
                    if (message != null) {
                        try {
                            this.currentClientOut.write(message);
                            // this.currentClientOut.flush();
                        } catch (IOException e) {
                            System.out.print("[wts] IOException: ");
                            System.out.println(e.getMessage());
                            break;
                        }
                    }
                    Thread.sleep(3); // bad, use blocking queue instead
                } catch (InterruptedException e) {
                    System.out.println("[wts] InterruptedException");
                }
            }

            System.out.println("Web telemetry streamer: Client disconnected !");
            channelMapSent = false;
        }
    }

    private void sendChannelMap() throws IOException {
        // Format: [marker byte 0xFF][num channels uint16][channel 0: uint16 name length][name string]...[channel id uint32]...
        ByteBuffer buffer = ByteBuffer.allocate(65536);
        
        // Marker byte to indicate this is a channel map
        buffer.put((byte) 0xFF);
        
        // Number of channels
        buffer.putShort((short) channelMap.size());
        
        // For each channel, write: uint16 name_length, string name, uint32 channel_id
        for (Map.Entry<String, Integer> entry : channelMap.entrySet()) {
            String name = entry.getKey();
            int id = entry.getValue();
            byte[] nameBytes = name.getBytes(StandardCharsets.UTF_8);
            
            buffer.putShort((short) nameBytes.length);
            buffer.put(nameBytes);
            buffer.putInt(id);
        }
        
        // Write the buffer
        int length = buffer.position();
        currentClientOut.write(buffer.array(), 0, length);
        currentClientOut.flush();
        
        System.out.println("[wts] Sent channel map with " + channelMap.size() + " channels");
    }

    public void sendData(String key, double value) {
        // Get or assign channel ID
        int channelId = channelMap.computeIfAbsent(key, k -> nextChannelId.getAndIncrement());
        
        // If this is a new channel and a client is connected, we should send updated map
        // For now, we'll just send the data with the channel ID
        
        // Binary format: uint16 length, int32 channel_id, double value
        ByteBuffer buffer = ByteBuffer.allocate(2 + 4 + 8);
        buffer.putShort((short) (4 + 8)); // length of (channel_id + value)
        buffer.putInt(channelId);
        buffer.putDouble(value);
        
        this.messageQueue.add(buffer.array());
    }

}
