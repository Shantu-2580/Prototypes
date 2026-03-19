from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import urlparse, parse_qs

HOST = "0.0.0.0"
PORT = 8000

class ESPHandler(BaseHTTPRequestHandler):

    def do_GET(self):
        parsed = urlparse(self.path)

        if parsed.path == "/move":
            params = parse_qs(parsed.query)

            left = params.get("L", ["0"])[0]
            right = params.get("R", ["0"])[0]

            print(f"🚗 MOVE → Left: {left} | Right: {right}")

        elif parsed.path == "/turn_left":
            print("↩ TURN LEFT 90°")

        elif parsed.path == "/turn_right":
            print("↪ TURN RIGHT 90°")

        else:
            print("❓ Unknown request:", self.path)

        # Send response
        self.send_response(200)
        self.send_header("Content-type", "text/plain")
        self.end_headers()
        self.wfile.write(b"OK")

def run():
    server = HTTPServer((HOST, PORT), ESPHandler)
    print(f"📡 ESP Simulator running on http://localhost:{PORT}")
    server.serve_forever()

if __name__ == "__main__":
    run()