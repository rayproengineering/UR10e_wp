import numpy as np
import itertools
from itertools import *
A=np.array([[x*np.sin(y) for y in range(1,41)] for x in range(1,101)])

b=set([0,1,2,3,4,5])

for (u,v) in product(b,b):
    print(u,v)

A=np.zeros((40,100))
print(A.size)
for u in range(len(A)):
    for v in range(len(u)):
        A[u,v]=
