from numpy import pi
import matplotlib.pyplot as plt
from pyclothoids import Clothoid, SolveG2




def f1():
    clothoid0 = Clothoid.G1Hermite(0, 0, pi, 1, 1, 0)
    #fig, ax = plt.subplots()
    print(clothoid0.SampleXY(500)[0])
    plt.plot(*clothoid0.SampleXY(500))
    plt.show()
    #plt.plot( *clothoid0.SampleXY(500) )
    print(clothoid0.dk, clothoid0.KappaStart, clothoid0.KappaEnd)

def f2():
    clothoid_list = SolveG2(0, 0, pi, 0, 1, 1, 0, 0)
    plt.figure()
    for i in clothoid_list:
        plt.plot( *i.SampleXY(500) )
    plt.show()

f2()

