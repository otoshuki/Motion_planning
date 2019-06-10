import time
i = 0
f = open('data.txt', 'w')
while True:
    a = time.time()
    i += 1
    f.seek(0)
    f.write(str(i))
    b = time.time()
    print(b-a)
