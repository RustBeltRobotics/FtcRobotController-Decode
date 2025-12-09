package org.firstinspires.ftc.teamcode;

import java.io.DataOutputStream;
import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;
import java.nio.charset.StandardCharsets;
import java.util.HashMap;
import java.util.Map;

public class WebTelemetryStreamer implements Runnable {

    ServerSocket serverSocket;

    Socket currentClient;

    DataOutputStream currentClientOut;

    int port = 8082;



    @Override
    public void run() {
        try {
            System.out.print("Web telemetry streamer listening on port");
            System.out.println(this.port);
            listen(this.port);
        } catch (IOException e) {
            System.out.print("Uh oh:");
            System.out.println(e.getMessage());
        }
    }

    WebTelemetryStreamer(int port) {
        this.port = port;
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


        System.out.print("Web telemetry streamer: Client connected");
    }

    public void stop() throws IOException {
        this.serverSocket.close();
    }

    private void listen(int port) throws IOException {
        this.serverSocket = new ServerSocket(port);

        while (true) {
            waitForClient();
            // bad
            while (currentClient.isConnected()) {
                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {

                }
            }
        }
    }

    public void sendData(String key, double value) {
        try {
            if (this.currentClient != null && this.currentClient.isConnected() && !this.currentClient.isOutputShutdown()) {
                this.currentClientOut.write(("event: data\r\n" + String.format("data: {\"%s\": %f}\r\n", key, value) + "\r\n").getBytes(StandardCharsets.UTF_8));
//                this.currentClientOut.flush();
            }
        } catch (IOException e) {
            System.out.println("sendData failed:");
            System.out.println(e.getMessage());
        }
    }

}
