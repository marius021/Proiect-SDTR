import socket

# Configurare server
HOST = '0.0.0.0'  # Ascultă pe toate interfețele
PORT = 8080       # Portul de ascultare

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    print(f"Server listening on {HOST}:{PORT}")

    while True:
        conn, addr = s.accept()
        with conn:
            print(f"Connected by {addr}")
            data = conn.recv(1024)
            if data:
                print(f"Received: {data.decode()}")
