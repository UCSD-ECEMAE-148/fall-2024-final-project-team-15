from google_auth_oauthlib.flow import Flow
from google.auth.transport.requests import Request
import os
import json

SCOPES = ['https://www.googleapis.com/auth/drive.file']

def authenticate():
    creds = None
    # Check if token.json exists for previously stored credentials
    if os.path.exists('token.json'):
        from google.oauth2.credentials import Credentials
        creds = Credentials.from_authorized_user_file('token.json', SCOPES)

    if not creds or not creds.valid:
        if creds and creds.expired and creds.refresh_token:
            creds.refresh(Request())
        else:
            # Use Web Application flow
            flow = Flow.from_client_secrets_file(
                'creds.json', SCOPES, redirect_uri='http://localhost'
            )
            auth_url, _ = flow.authorization_url(prompt='consent')
            print("Authorization URL:", auth_url)

            # Ask user to manually paste authorization code
            auth_code = input("Enter the authorization code: ")
            flow.fetch_token(code=auth_code)
            creds = flow.credentials

        # Save the credentials to token.json
        with open('token.json', 'w') as token_file:
            token_file.write(creds.to_json())

    return creds

if __name__ == "__main__":
    authenticate()
