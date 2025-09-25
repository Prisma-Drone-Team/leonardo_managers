import requests

def get_stream():
    url = "http://127.0.0.1:5000/mission_updates"
    headers = {"Client-ID": "test_client"}
    #response = requests.get(url, stream=True)
    response = requests.get(url, headers=headers, stream=True)
    for line in response.iter_lines():
        if line:
            print(line.decode('utf-8'))

if __name__ == "__main__":
    try:
        try:
            get_stream()
        except requests.exceptions.ConnectionError:
            print("Connection error: Unable to connect to the server. Please ensure the server is running.")
            exit()
    except KeyboardInterrupt:
        print("\nStream interrupted by user. Exiting...")
        exit()