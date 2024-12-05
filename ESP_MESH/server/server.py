import socket

def start_server(host='0.0.0.0', port=5555):
    """Start a simple TCP server."""
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    try:
        server_socket.bind((host, port))
        server_socket.listen(5)
        print(f"Server listening on {host}:{port}...")

        while True:
            client_socket, client_address = server_socket.accept()
            print(f"Connection from {client_address}")

            # Handle client request
            data = client_socket.recv(1024)
            if data:
                print(f"Received: {data.decode('utf-8')}")
                client_socket.sendall(b"Hello from the server!\n")
            
            client_socket.close()

    except Exception as e:
        print(f"Error: {e}")
    finally:
        server_socket.close()
        print("Server shut down.")

if __name__ == "__main__":
    start_server()
