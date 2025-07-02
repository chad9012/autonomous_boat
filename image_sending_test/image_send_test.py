from flask import Flask, request, render_template_string
import os

app = Flask(__name__)

UPLOAD_FOLDER = 'image_sending_test/uploads'
os.makedirs(UPLOAD_FOLDER, exist_ok=True)

HTML_FORM = """
<!doctype html>
<title>Upload Photo</title>
<h1>Upload Photo</h1>
<form method=post enctype=multipart/form-data>
  <input type=file name=photo>
  <input type=submit value=Upload>
</form>
"""

@app.route('/', methods=['GET', 'POST'])
def upload_photo():
    if request.method == 'POST':
        if 'photo' not in request.files:
            return 'No file part'
        file = request.files['photo']
        if file.filename == '':
            return 'No selected file'
        filepath = os.path.join(UPLOAD_FOLDER, file.filename)
        file.save(filepath)
        return f'File saved as: {filepath}'
    return render_template_string(HTML_FORM)

if __name__ == '__main__':
    app.run(port=5000)
