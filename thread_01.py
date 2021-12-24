import threading
import time

def first_fun(age):
    while True:
        print('I am first child.', age)
        time.sleep(1)
    return

def second_fun(age):
    while True:
        print('I am second child.', age)
        time.sleep(1)
    return

if __name__ == '__main__':
    first = threading.Thread(target=first_fun, args=(5,))
    second = threading.Thread(target=second_fun, args=(3,))
    first.start()
    second.start()
    first.join()
    second.join()

    while True :
        print('I am parent.')
    pass