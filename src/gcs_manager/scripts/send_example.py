import requests

url = "http://127.0.0.1:5000/upload_file"
file_path = "../../images/image.png"
metadata = {
            "task_id": 13,               
            "target_type": "aruco",
            "target_id": "123",    
            "request_time": "2025/10/23 00:00:00",
            "completion_time": "2025/10/23 00:00:00",    
            "result": [
                        "sector_1", # find target
                        "None", # find target QR
                        "None", # find object
                        "None", # follow sequence
                        "None", # land highest spot
                        "None", # emergency landing
                        "None", # return to base
                      ] }

files = {
    'file': open(file_path, 'rb'),
    'metadata': (None, str(metadata), 'application/json')
}

files_missing_metadata = {
    'file': open(file_path, 'rb'),
    'metadata': None  # Simulate missing metadata
}



if __name__ == "__main__":
    try:
        headers = {"Client-ID": "test_client"}                      # Attach an optional client ID header
        response = requests.post(url, headers=headers, files=files) # Send the file and metadata
        print(response.json())

        msg = metadata
        response = requests.post(url, headers=headers, json=msg)  # Send metadata as JSON message
        print(response.json())

        response = requests.post(url, headers=headers, files=files_missing_metadata) # Send the file missing metadata (returns server error response)
        print(response.json())

        msg = metadata
        response = requests.post(url, headers=headers, json="Nota a JSON")  # Send invalid JSON message (will be uploaded nevertheless)
        print(response.json())
        
    except requests.exceptions.RequestException as e:
        print(f"An error occurred: {e}")
