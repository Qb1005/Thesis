import math

import matplotlib.pyplot as plt
import time
show_animation = True
def main():
        # set obstacle positions
    ox, oy = [], []
    for i in range(-10, 60):
            ox.append(i)
            oy.append(-10.0)
    for i in range(-10, 60):
            ox.append(60.0)
            oy.append(i)
    for i in range(-10, 61):
            ox.append(i)
            oy.append(60.0)
    for i in range(-10, 61):
            ox.append(-10.0)
            oy.append(i)
    for i in range(0, 10):
            ox.append(0.0)
            oy.append(i)
    for i in range(0, 11):
            ox.append(5.0)
            oy.append(i)
    for i in range(0, 5):
            ox.append(i)
            oy.append(0)
    for i in range(0, 5):
            ox.append(i)
            oy.append(10)

    for i in range(20, 31):
            ox.append(0.0)
            oy.append(i)
    for i in range(20, 31):
            ox.append(5.0)
            oy.append(i)
    for i in range(0, 5):
            ox.append(i)
            oy.append(20)
    for i in range(0, 5):
            ox.append(i)
            oy.append(30)   

    for i in range(40, 51):
            ox.append(0.0)
            oy.append(i)
    for i in range(40, 51):
            ox.append(5.0)
            oy.append(i)
    for i in range(0, 5):
            ox.append(i)
            oy.append(40)
    for i in range(0, 5):
            ox.append(i)
            oy.append(50)  

        ##
    for i in range(0, 11):
            ox.append(20.0)
            oy.append(i)
    for i in range(0, 11):
            ox.append(25.0)
            oy.append(i)
    for i in range(20, 25):
            ox.append(i)
            oy.append(0)
    for i in range(20, 25):
            ox.append(i)
            oy.append(10)

    for i in range(20, 31):
            ox.append(20.0)
            oy.append(i)
    for i in range(20, 31):
            ox.append(25.0)
            oy.append(i)
    for i in range(20, 25):
            ox.append(i)
            oy.append(20)
    for i in range(20, 25):
            ox.append(i)
            oy.append(30)   

    for i in range(40, 51):
            ox.append(20.0)
            oy.append(i)
    for i in range(40, 51):
            ox.append(25.0)
            oy.append(i)
    for i in range(20, 25):
            ox.append(i)
            oy.append(40)
    for i in range(20, 25):
            ox.append(i)
            oy.append(50)  

    ##
    for i in range(0, 11):
            ox.append(40.0)
            oy.append(i)
    for i in range(0, 11):
            ox.append(45.0)
            oy.append(i)
    for i in range(40, 45):
            ox.append(i)
            oy.append(0)
    for i in range(40, 45):
            ox.append(i)
            oy.append(10)

    for i in range(20, 31):
            ox.append(40.0)
            oy.append(i)
    for i in range(20, 31):
            ox.append(45.0)
            oy.append(i)
    for i in range(40, 45):
            ox.append(i)
            oy.append(20)
    for i in range(40, 45):
            ox.append(i)
            oy.append(30)   

    for i in range(40, 51):
            ox.append(40.0)
            oy.append(i)
    for i in range(40, 51):
            ox.append(45.0)
            oy.append(i)
    for i in range(40, 45):
            ox.append(i)
            oy.append(40)
    for i in range(40, 45):
            ox.append(i)
            oy.append(50)

    for i in range (-10,20):
        ox.append(i)
        oy.append(35)

    for i in range (30,60):
        ox.append(i)
        oy.append(15)

    for i in range (-10,9):
        ox.append(12)
        oy.append(i)

    for i in range (15,50):
        ox.append(30)
        oy.append(i)

    plt.plot(ox, oy, ".k")
    plt.pause(0.01)
    plt.show()

        

if __name__ == '__main__':
    main()
    