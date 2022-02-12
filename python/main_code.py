#!/bin/python

import modules.module1 as m1
from modules import module2 as m2
#from modules import module3 as m3
import modules
#from modules import module2 module3 as m2 m3   --- nao funciona

m1.fun1()
m1.fun3()
m1.fun2()

m2.fun2()
m2.fun3()
m2.fun1()

modules.module3.fun3()
