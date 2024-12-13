import rclpy
from rclpy.node import Node
import requests
import os
from google.oauth2.credentials import Credentials
from googleapiclient.discovery import build
from googleapiclient.http import MediaFileUpload

class DataUploader(Node):
    def __init__(self):
        super().__init__('data_uploader')
        # Declare parameters with default values
        self.declare_parameter('folder_path', '/home/projects/ros2_ws/src/final_project/Upload_Folder')
        self.declare_parameter('gdrive_folder_id', '1BadjokRd84xGZLxXlP32D_2VlA5RM3MQ')

        # Get parameter values
        self.folder_path = self.get_parameter('folder_path').value
        self.gdrive_folder_id = self.get_parameter('gdrive_folder_id').value

        # Timer to check internet and process files every 10 seconds
        self.timer = self.create_timer(10, self.check_internet)

    def check_internet(self):
        """Checks if there is an active internet connection."""
        try:
            response = requests.get('https://www.google.com', timeout=5)
            if response.status_code == 200:
                self.get_logger().info('Internet is connected.')
                self.upload_folder_to_drive()
            else:
                self.get_logger().warn('Internet is not connected.')
        except requests.ConnectionError:
            self.get_logger().warn('Internet is not connected.')

    def authenticate_gdrive(self):
        """Authenticates with Google Drive API using token.json."""
        SCOPES = ['https://www.googleapis.com/auth/drive.file']
        try:
            creds = Credentials.from_authorized_user_file('/home/projects/ros2_ws/src/final_project/token.json', SCOPES)
            if not creds or not creds.valid:
                raise Exception("Invalid credentials. Re-run the OAuth flow.")
            return build('drive', 'v3', credentials=creds)
        except Exception as e:
            self.get_logger().error(f"Authentication failed: {e}")
            return None

    def upload_folder_to_drive(self):
        """Uploads all files in the specified folder to Google Drive."""
        service = self.authenticate_gdrive()
        if not service:
            self.get_logger().error("Unable to authenticate Google Drive service.")
            return

        # Check if the folder exists and contains files
        if os.path.exists(self.folder_path) and os.listdir(self.folder_path):
            self.get_logger().info(f"Uploading files from folder: {self.folder_path}")
            for file_name in os.listdir(self.folder_path):
                file_path = os.path.join(self.folder_path, file_name)
                if os.path.isfile(file_path):
                    self.upload_file_to_drive(file_path, service)
            self.cleanup_folder()
        else:
            self.get_logger().info("No files to upload.")

    def upload_file_to_drive(self, file_path, service):
        """Uploads a single file to Google Drive."""
        try:
            file_metadata = {
                'name': os.path.basename(file_path),
                'parents': [self.gdrive_folder_id]  # Use the parameter value
            }
            media = MediaFileUpload(file_path, resumable=True)
            file = service.files().create(
                media_body=media, body=file_metadata, fields='id').execute()
            self.get_logger().info(f"Uploaded file: {file_path} with File ID: {file.get('id')}")
        except Exception as e:
            self.get_logger().error(f"Failed to upload file {file_path}. Error: {e}")

    def cleanup_folder(self):
        """Removes all files from the folder after successful upload."""
        try:
            for file_name in os.listdir(self.folder_path):
                file_path = os.path.join(self.folder_path, file_name)
                if os.path.isfile(file_path):
                    os.remove(file_path)
            self.get_logger().info(f"Cleared folder: {self.folder_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to clean up folder. Error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = DataUploader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

