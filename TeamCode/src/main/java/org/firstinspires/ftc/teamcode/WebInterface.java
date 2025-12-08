package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.util.WebServer;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOError;
import java.io.IOException;
import java.net.ServerSocket;

import java.nio.channels.ServerSocketChannel;

import java.net.Socket;
import java.nio.charset.StandardCharsets;
import java.util.Arrays;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

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
        try {
            listen(this.port);
        } catch (IOException e) {

        }
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

    private void listen(int port) throws IOException {
        this.mode = Mode.LISTEN;
        this.serverSocket = new ServerSocket(port);

        while (true) {
            Socket client = this.serverSocket.accept();
            handleConnection(client);
            client.close();
        }
    }

    private void handleConnection(Socket client) throws IOException {
        DataInputStream d = new DataInputStream(client.getInputStream());
        String request = d.readUTF();
        String[] lines = request.split("(\r\n)|\n");

        String requestLine = lines[0];
        Pattern pattern = Pattern.compile("([A-Z]+) ([^\\s\\r\\n]+) ([^\\s\\r\\n]+)");
        Matcher matched = pattern.matcher(requestLine);

        String method = matched.group(1);
        String route = matched.group(2);
        String protocol = matched.group(3);

        String[] headers = Arrays.copyOfRange(lines, 1, lines.length);
        String response = handleRoute(route, method, protocol, headers);

        DataOutputStream o = new DataOutputStream(client.getOutputStream());
        o.write(response.getBytes(StandardCharsets.UTF_8));
        o.flush();
    }

    private String handleRoute(String route, String method, String protocol, String[] headers) {
        String[] bla = route.split("\\?", 1);
        String justRoute = bla[0];
        String search = bla[1];

        if (method.equals("GET")) {
            switch (justRoute) {
                case "/":

                    return "HTTP/1.1 200 OK\r\n\r\n" + indexHTML();
                case "/setValue":
                    String[] split = search.split(":");
                    parameters.put(split[0], Double.parseDouble(split[1]));
                    return "HTTP/1.1 200 OK\r\n\r\n";
                default:
                    return "HTTP/1.1 404 Not Found\r\n\r\n";
            }
        }

        return "HTTP/1.1 400 Bad Request\r\n\r\n";
    }


    private String indexHTML() {
        String sliders = "";
        for (Map.Entry<String, Double> entry : parameters.entrySet()) {
            sliders += String.format("<label>%s:</label><input type=\"slider\" value=\"%d\" /><br/>", entry.getKey(), entry.getValue());
        }

        return "<!DOCTYPE html><html lang=\"en\"><head><title>FTC Web Interface</title></head><body>\n"
               + sliders
               +"</body></html>";
    }
}
