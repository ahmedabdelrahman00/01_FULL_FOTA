from flask import Flask, render_template, request, jsonify, redirect, url_for, session
from flask import Flask, render_template, request, jsonify, redirect, url_for, session

import firebase_admin
from firebase_admin import credentials, storage
import os
app = Flask(__name__, template_folder='.')


# Use credentials.Certificate with the path to the Firebase Admin SDK credentials file (fota-d43b8-firebase-adminsdk-os3z7-38c8441d6a.json)
cred = credentials.Certificate(
    "full-fota-firebase-adminsdk-97lkk-d3c723d61a.json")
firebase_admin.initialize_app(
    cred, {'storageBucket': 'full-fota.appspot.com'})
bucket = storage.bucket()

print("Template Folder Path:", os.path.abspath(app.template_folder))


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/login', methods=['GET', 'POST'])
def login():
    if request.method == 'POST':
        username = request.form.get('username')
        password = request.form.get('password')

        # Replace the following with your authentication logic
        if username == 'admin' and password == '12345':
            session['logged_in'] = True
            # return redirect(url_for('file_upload_form'))
            return redirect('/file-upload')

        else:
            return render_template('login.html', error='Invalid credentials')

    return render_template('file-upload.html')


@app.route('/file-upload.html')
def file_upload_form():
    if 'logged_in' in session and session['logged_in']:
        return render_template('file-upload.html')
    else:
        return redirect(url_for('login'))


@app.route('/upload', methods=['POST'])
def upload():
    if 'file' not in request.files:
        return jsonify({'error': 'No file part'})

    file = request.files['file']

    if file.filename == '':
        return jsonify({'error': 'No selected file'})

    try:
        # Upload the file to Firebase Storage
        blob = bucket.blob('uploaded_file.txt')
        blob.upload_from_file(file)
        return jsonify({'message': 'File uploaded successfully'})
    except Exception as e:
        return jsonify({'error': str(e)})


if __name__ == '__main__':
    app.run(debug=True)
