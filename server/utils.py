# utils for saving pgm/ppm files
def write_pgm(filepath, bitmap, w, h):
    with open(filepath, 'wb') as f:
        f.write("P5\n{}\t{}\n65535\n".format(w, h).encode('ascii'))
        f.write(bitmap.tobytes())

def write_ppm(filepath, bitmap, w, h):
    with open(filepath, 'wb') as f:
        f.write("P6\n{}\t{}\n255\n".format(w, h).encode('ascii'))
        f.write(bitmap.tobytes())
