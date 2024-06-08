from time import sleep

while True:
    milliseconds = int(input('enter the time in milliseconds: '))
    print(milliseconds / 1000)
    for i in range(1, milliseconds + 1):
        print(i)
        sleep(0.001)
    print('finished')