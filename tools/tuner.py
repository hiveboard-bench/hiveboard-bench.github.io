import http.server
import socketserver
import json
import re
import subprocess

PORT = 8081
FILE_PATH = "tools/build-sim-assets.py"

class TunerHandler(http.server.SimpleHTTPRequestHandler):
    def do_OPTIONS(self):
        self.send_response(200, "ok")
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()

    def do_POST(self):
        if self.path == '/build':
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length)
            params = json.loads(post_data.decode('utf-8'))
            
            with open(FILE_PATH, 'r') as f:
                content = f.read()
                
            for key, val in params.items():
                content = re.sub(rf'("{key}":\s*)[0-9.-]+', rf'\g<1>{val}', content)
                
            with open(FILE_PATH, 'w') as f:
                f.write(content)
                
            try:
                result = subprocess.run(["python3", FILE_PATH], capture_output=True, text=True)
                log = result.stdout + "\n" + result.stderr
            except Exception as e:
                log = str(e)
                
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps({"log": log}).encode())

class ThreadingTCPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = True

with ThreadingTCPServer(("", PORT), TunerHandler) as httpd:
    print(f"Tuner API rodando na porta {PORT}")
    httpd.serve_forever()
