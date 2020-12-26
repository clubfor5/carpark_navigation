from matplotlib import pyplot as plt 
result = [3.29,2.91,4.51,4.02,3.26]
plt.bar(range(len(result)), result)
x = [-0.5,4.5]
y = [2.7,2.7]
optimal = plt.plot(x,y,color='r',linestyle='--')
#plt.legend([optimal],['optimal parameter settings'])
plt.show()