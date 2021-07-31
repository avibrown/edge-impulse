from pydub.utils import make_chunks
from pydub import AudioSegment
import os

export_path = './test/'

def split_audio(filename):
    file = AudioSegment.from_file(filename, "wav")
    chunk_length = 1000
    chunks = make_chunks(file, chunk_length)
    
    for i, chunk in enumerate(chunks):
        chunk_name = export_path + filename + f"_{i}.wav"
        print ("Exporting: ", chunk_name)
        chunk.export(chunk_name, format="wav")

if not os.path.isdir(export_path):
    os.makedirs(export_path)
    
files = os.listdir('./')

for file in files:
    print(file)
    if ('.wav' in file):
        split_audio(file)