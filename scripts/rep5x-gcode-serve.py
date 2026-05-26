#!/usr/bin/env python3
"""
One-shot CORS HTTP server for handing a .gcode file off to the Rep5x web viewer.

Usage: rep5x-gcode-serve.py <directory>
Prints the chosen port to stdout, then spawns a background process that serves
exactly one HTTP request (the viewer's GET of the .gcode), then exits.

Works on Linux, macOS, and Windows (uses multiprocessing instead of os.fork
so there is no fork() requirement).
"""
import http.server
import multiprocessing
import os
import socket
import socketserver
import sys

class CORSHandler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        super().end_headers()
    def log_message(self, fmt, *args):
        pass  # quiet

def serve_one(serve_dir, port):
    os.chdir(serve_dir)
    # Re-bind in the child using the port the parent picked.
    httpd = socketserver.TCPServer(('127.0.0.1', port), CORSHandler)
    httpd.timeout = 180
    try:
        httpd.handle_request()
    finally:
        httpd.server_close()

def main():
    serve_dir = sys.argv[1] if len(sys.argv) > 1 else '.'
    # Pick a free port up-front so we can print it before spawning the worker.
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(('127.0.0.1', 0))
    port = s.getsockname()[1]
    s.close()
    print(port, flush=True)

    # Spawn the worker as a fully detached background process and return.
    p = multiprocessing.Process(target=serve_one, args=(serve_dir, port), daemon=False)
    p.start()
    # Parent exits immediately; child keeps the port and serves one request.

if __name__ == '__main__':
    # Required on Windows so the child process re-imports this module cleanly.
    multiprocessing.freeze_support()
    main()
