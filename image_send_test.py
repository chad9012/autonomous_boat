from flask import Flask, request, redirect, url_for, render_template_string
import os

app = Flask(__name__)
UPLOAD_FOLDER = 'uploads'
os.makedirs(UPLOAD_FOLDER, exist_ok=True)

HTML_FORM = '''
<!doctype html>
<title>Upload Photo</title>
<h1>Upload Photo</h1>
<form method=post enctype=multipart/form-data>
  <input type=file name=photo>
  <input type=submit value=Upload>
</form>
'''

@app.route('/', methods=['GET', 'POST'])
def upload_file():
    if request.method == 'POST':
        if 'photo' not in request.files:
            return 'No file part'
        file = request.files['photo']
        if file.filename == '':
            return 'No selected file'
        if file:
            filepath = os.path.join(UPLOAD_FOLDER, file.filename)
            file.save(filepath)
            return f'File uploaded: {file.filename}'
    return render_template_string(HTML_FORM)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
