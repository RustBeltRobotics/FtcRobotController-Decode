package org.firstinspires.ftc.teamcode;

import java.io.DataOutputStream;
import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;
import java.nio.charset.StandardCharsets;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ConcurrentLinkedQueue;

public class WebTelemetryStreamer implements Runnable {

    ServerSocket serverSocket;

    Socket currentClient;

    DataOutputStream currentClientOut;

    ConcurrentLinkedQueue<String> messageQueue;

    int port = 8082;



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
                        "Content-Type: text/event-stream\r\n"+
                        "Cache-Control: no-cache\r\n"+
                        "Connection: keep-alive\r\n"+
                        "Access-Control-Allow-Origin: *\r\n"+
                        "\r\n0\r\n";


        this.currentClientOut = new DataOutputStream(currentClient.getOutputStream());
        currentClientOut.write(response_start.getBytes(StandardCharsets.UTF_8));
        currentClientOut.flush();


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
                    String message = messageQueue.poll();
//                    System.out.println("[wts] mq poll");
                    if (message != null) {
                        try {
                            this.currentClientOut.write(message.getBytes(StandardCharsets.UTF_8));
                            // this.currentClientOut.flush();
                        } catch (IOException e) {
                            // nothing
                            System.out.print("[wts] IOException: ");
                            System.out.println(e.getMessage());
                            break;
                        }
                    } else {
                        System.out.print("[wts] Message null");
                    }
                    Thread.sleep(3); // bad, use blocking queue instead
                } catch (InterruptedException e) {
                    System.out.print("[wts] InterruptedException");
                }
            }

            System.out.println("Web telemetry streamer: Client disconnected !");
        }
    }

    public void sendData(String key, double value) {
//        if (this.currentClient != null && this.currentClient.isConnected() && !this.currentClient.isOutputShutdown()) {
            this.messageQueue.add(("event: data\r\n" + String.format("data: {\"%s\": %f}\r\n", key, value) + "\r\n"));
//        }
    }

}
