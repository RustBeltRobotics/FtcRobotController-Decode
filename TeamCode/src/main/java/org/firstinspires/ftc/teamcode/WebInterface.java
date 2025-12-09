package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.util.WebServer;

import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOError;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.ServerSocket;

import java.nio.channels.ServerSocketChannel;

import java.net.Socket;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
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
            System.out.print("Listening on port");
            System.out.println(this.port);
            listen(this.port);
        } catch (IOException e) {
            System.out.print("Uh oh:");
            System.out.println(e.getMessage());
        }
    }

    WebInterface(int port) {
        this.parameters = new HashMap<String, Double>();
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

    public void stop() throws IOException {
        System.out.println("Stopping web interface...");
        this.serverSocket.close(); // TODO: make this more graceful
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
        System.out.println("New connection");
        BufferedReader d = new BufferedReader(new InputStreamReader(client.getInputStream(), StandardCharsets.UTF_8));
//.readUTF(); // oops this is modified utf-8 apparently
        // oops this was not reading all lines (it just saved all lines that have been sent so far, 0 in my testing, as it's called almost instantly after the connection is started)
//        String request = d.toString();
//        String[] lines = request.split("(\r\n)|\n");

        // read all lines:
        ArrayList<String> linesar = new ArrayList<>();
        String line;
        while ((line = d.readLine()) != null && !line.isEmpty()) {
            linesar.add(line);
        }

        String[] lines = linesar.toArray(new String[0]);

        String requestLine = lines[0];

        System.out.print("Req: ");
        System.out.println(requestLine);
        Pattern pattern = Pattern.compile("([A-Z]+) ([^\\s\\r\\n]+) ([^\\s\\r\\n]+).+");
        Matcher matched = pattern.matcher(requestLine);

        if (!matched.matches()) {
            System.out.println("Parsing failed, no match.");
            String response = "HTTP/1.1 400 Bad Request\r\n\r\nparsing failed";
            DataOutputStream o = new DataOutputStream(client.getOutputStream());
            o.write(response.getBytes(StandardCharsets.UTF_8));
            o.flush();

            System.out.println("sent failure response");
            return;
        }

        String method = matched.group(1);
        String route = matched.group(2);
        String protocol = matched.group(3);

//        String[] headers = Arrays.copyOfRange(lines, 1, lines.length);
        String[] headers = {}; // temporarily disabled because it was crashing here I think

        String response = handleRoute(route, method, protocol, headers);

        DataOutputStream o = new DataOutputStream(client.getOutputStream());
        o.write(response.getBytes(StandardCharsets.UTF_8));
        o.flush();


        System.out.println("sent response, done.");
    }

    private String handleRoute(String route, String method, String protocol, String[] headers) {
        String[] bla = route.split("\\?", 2);
        String justRoute = bla[0];
        String search = bla.length > 1 ? bla[1] : "";

        if (method.equals("GET")) {
            switch (justRoute) {
                case "/":
                    return "HTTP/1.1 200 OK\r\n\r\n" + indexHTML();
                case "/setValue":
                    String[] split = search.split(":");
                    parameters.put(split[0], Double.parseDouble(split[1]));
                    System.out.print("Set parameter ");
                    System.out.print(split[0]);
                    System.out.print(" to ");
                    System.out.println(Double.parseDouble(split[1]));
                    return "HTTP/1.1 200 OK\r\n\r\n";
                default:
                    return "HTTP/1.1 404 Not Found\r\n\r\n";
            }
        }

        return "HTTP/1.1 400 Bad Request\r\n\r\n";
    }


    private String indexHTML() {
        String sliders = "";
        String sliderArray = "[";
        for (Map.Entry<String, Double> entry : parameters.entrySet()) {
            double value = entry.getValue();
            String key = entry.getKey();
            sliders += String.format("<label>%s:</label><input id=\"%s\" type=\"range\" min=\"0.0\" max=\"2.0\" step=\"0.01\" value=\"%f\" /><label id=\"%s_l\">%f</label><br/>", key, key, value, key, value);
            sliderArray += "\"" + key + "\",";
        }

        String js = sliderArray + "].forEach( key => document.getElementById(key).addEventListener('input', (ev) => {  document.getElementById(key+'_l').innerText=ev.target.value;  fetch('/setValue?'+ev.target.id+':'+ev.target.value)  }));" +
                "const evs = new EventSource(\"http://\" + document.location.hostname + ':8886'); evs.addEventListener('data', (ev) => {applyUpdate(JSON.parse(ev.data))});"+
                "const canvas = document.getElementById(\"canvas\");"+
                "const ctx = canvas.getContext(\"2d\");\n\nconst series = {};\nconst seriesColors = {};\nconst startTime=Date.now();\n\nfunction randomColor() {\n    return `hsl(${Math.floor(Math.random() * 360)}, 80%, 50%)`;\n}\n\nfunction applyUpdate(updateObj) {\n    const keys = Object.keys(updateObj);\n    if (keys.length === 0) return;\n\n    const key = keys[0];\n    const value = updateObj[key];\n\n    if (!series[key]) series[key] = [];\n    if (!seriesColors[key]) seriesColors[key] = randomColor();\n\n    series[key].push({ x: Date.now(), y: value });\n\n    drawGraph();\n}\n\nfunction drawGraph() {\n    ctx.clearRect(0, 0, canvas.width, canvas.height);\n\n    for (const key in series) {\n        const points = series[key];\n        const color = seriesColors[key];\n\n        ctx.beginPath();\n        ctx.strokeStyle = color;\n\n        for (let i = 0; i < points.length; i++) {\n            const p = points[i];\n            const canvasY = canvas.height - p.y;\nconst xp = (p.x-startTime)-canvas.width/2;            if (i === 0) {\n                ctx.moveTo(xp, canvasY);\n            } else {\n                ctx.lineTo(xp, canvasY);\n            }\n        }\n\n        ctx.stroke();\n    }\n}\n";

        return "<!DOCTYPE html><html lang=\"en\"><head><title>FTC Web Interface</title></head><body>\n"
               + sliders
               +"<canvas id=\"canvas\" width=\"800\" height=\"450\"></canvas>"
               +"<script>" + js + "</script></body></html>";
    }
}
