import socket
from pydub.generators import Sine

# Server configuration
HOST = '0.0.0.0'  # Listen on all available interfaces
PORT = 12345      # Port number (must match the ESP32's configuration)

# Generate a sine wave audio
frequency = 440.0  # A4 note
duration = 30 * 1000  # 30 seconds in milliseconds
audio = Sine(frequency).to_audio_segment(duration=duration)

# Save as AAC
AUDIO_FILE = "test.aac"
audio.export(AUDIO_FILE, format="adts", bitrate="128k")
print(f"Generated AAC file saved as {AUDIO_FILE}")

# Create a TCP socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((HOST, PORT))
server_socket.listen(1)

print(f"Server listening on {HOST}:{PORT}")

try:
    while True:
        # Accept a connection from the client (ESP32)
        client_socket, client_address = server_socket.accept()
        print(f"Connection from {client_address}")

        # Open the AAC file in binary mode and send its contents
        with open(AUDIO_FILE, 'rb') as audio_file:
            print(f"Sending AAC file: {AUDIO_FILE}")
            data = audio_file.read(1440)  # Read in chunks
            while data:
                client_socket.sendall(data)
                data = audio_file.read(1440)

        print("Finished sending AAC file.")

        # Close the client connection
        client_socket.close()

except KeyboardInterrupt:
    print("Shutting down server...")
finally:
    server_socket.close()