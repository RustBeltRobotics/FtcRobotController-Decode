package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.util.WebServer;

import java.io.DataInputStream;
import java.io.IOError;
import java.io.IOException;
import java.net.ServerSocket;

import java.net.Socket;
import java.util.Map;

public class WebInterface {


    ServerSocket serverSocket;
    Map<String, Double> parameters;

    WebInterface() {

    }

    void addParameter(String key, double defaultValue) {
        parameters.put(key, defaultValue);
    }

    void addParameter(String key) {
        parameters.put(key, 0.0);
    }

    Double getParameter(String key) {
        return parameters.get(key);
    }

    void listen(int port) {
        try {
            this.serverSocket = new ServerSocket(port);

            Socket client = this.serverSocket.accept();


            DataInputStream d = new DataInputStream(client.getInputStream());
            String str = d.readUTF().replaceAll("[\\r\\n\\s]", "");
            String[] split = str.split(":");
            parameters.put(split[0], Double.parseDouble(split[1]));
            client.close();
        } catch (IOException err) {

        }
    }
}
