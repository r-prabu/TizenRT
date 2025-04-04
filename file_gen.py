filename = "test1.bin"

with open(filename, "wb") as f:
    # 20 MB file
    for i in range(1024) :
        for j in range(64 * 1024) :
            f.write(i.to_bytes(2, byteorder='little'))
