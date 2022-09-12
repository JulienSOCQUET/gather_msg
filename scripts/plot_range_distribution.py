import math
import numpy             as np
import matplotlib.pyplot as plt


def txt_to_data(path):
    lines = []
    data = []
    with open(path) as f:
        lines = f.readlines()
    for line in lines[1:]:
        data.append([float(item) for item in line.split(",")])
        
    return np.array(data)


def plot_data(data, label):
    min_points = 0 #100
    max_delta  = float('inf') #90
    resolution = 40
    
    data[:, 2] = np.sqrt(data[:, 2] - data[:, 1]**2)
    data       = np.delete(data, np.where((data[:, 4] - data[:, 3]) > max_delta), axis=0)
    data       = np.delete(data, np.where(data[:, 0] < min_points), axis=0)

    histo = np.zeros((math.floor(max(data[:, 1]) / resolution) + 1, 2))
    for ii in range(data.shape[0]):
        histo[int(data[ii, 1] // resolution), :] += np.array([1, data[ii, 2]])
        
    for ii in range(histo.shape[0]):
        if histo[ii, 0] != 0:
            histo[ii, 1] = histo[ii, 1] / histo[ii, 0]

    plt.scatter(data[:, 1], data[:, 2], label=label, s=5)
    #plt.plot(resolution * np.arange(0, histo.shape[0]), histo[:, 1], label=label)
    

data_512x10  = txt_to_data("2022-05-27-12-44-20_analyse_ouster.txt")
data_1024x10 = txt_to_data("2022-05-27-12-45-25_analyse_ouster.txt")
data_2048x10 = txt_to_data("2022-05-27-12-47-29_analyse_ouster.txt")
data_512x20  = txt_to_data("2022-05-27-12-48-54_analyse_ouster.txt")
data_1024x20 = txt_to_data("2022-05-27-12-49-52_analyse_ouster.txt")

plot_data(data_512x10,  "512x10")
plot_data(data_1024x10, "1024x10")
plot_data(data_2048x10, "2048x10")
plot_data(data_512x20,  "512x20")
plot_data(data_1024x20, "1024x20")

plt.legend(loc="upper left")
plt.xlabel("Mean range (mm)")
plt.ylabel("Std dev range (mm)")
plt.show()


