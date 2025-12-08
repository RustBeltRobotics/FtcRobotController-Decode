package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.util.WebServer;

import java.io.DataInputStream;
import java.io.IOError;
import java.io.IOException;
import java.net.ServerSocket;

import java.nio.channels.ServerSocketChannel;

import java.net.Socket;
import java.util.Map;


enum Mode {
    INITIALIZE,
    LISTEN
}

public class WebInterface implements Runnable {

    ServerSocket serverSocket;
    Map<String, Double> parameters;

    Mode mode; // prevent calling addParameter after listen is called, for thread safety

    int port = 8081;

    @Override
    public void run() {
        listen(this.port);
    }

    WebInterface(int port) {
        this.mode = Mode.INITIALIZE;
        this.port = port;
    }

    public void addParameter(String key, double defaultValue) {
        if (this.mode != Mode.INITIALIZE) return;
        parameters.put(key, defaultValue);
    }

    public void addParameter(String key) {
        if (this.mode != Mode.INITIALIZE) return;
        parameters.put(key, 0.0);
    }

    public Double getParameter(String key) {
        return parameters.get(key);
    }

    private void listen(int port) {
        this.mode = Mode.LISTEN;
        try {
            this.serverSocket = new ServerSocket(port);

            while (true) {
                Socket client = this.serverSocket.accept();
                DataInputStream d = new DataInputStream(client.getInputStream());
                String str = d.readUTF().replaceAll("[\\r\\n\\s]", "");
                String[] split = str.split(":");
                parameters.put(split[0], Double.parseDouble(split[1]));
                client.close();
            }
        } catch (IOException err) {

        }
    }
}
