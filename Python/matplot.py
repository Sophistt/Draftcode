import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import scipy as sp
from scipy.optimize import leastsq
from scipy import interpolate


def bezier(control_points, rank):
    temp_points = []
    planpath = []

    for i in range(200):
        u = i / 199.0
        for index in range(len(control_points)):
            temp_points.append(dict(control_points[index]))

        #print(i, control_points)

        j = 1
        while j <= rank:
            k = 0
            while k <= (rank - j):
                temp_points[k]["x"] = (
                    1-u) * temp_points[k]["x"] + u * temp_points[k+1]["x"]
                temp_points[k]["y"] = (
                    1-u) * temp_points[k]["y"] + u * temp_points[k+1]["y"]
                k += 1
            j += 1
        planpath.append(dict(temp_points[0]))

    return planpath


# ----------------------------------
# Used for curvilinear regression
# ----------------------------------
def curve(x, a, b, c, d):
    return a*x**3 + b*x**2 + c*x**1 + d


def error(p, x, y):
    a, b, c, d = p
    return curve(x, a, b, c, d) - y


def func_k(x, a, b, c):
    return 3*a*x**2 + 2*b*x + c


# ---------------------------------

def main():
    # Set 4 control points of bezier curve
    ap_dict = {"x": 390, "y": 200}
    ap1_dict = {"x": 390, "y": 240}
    local1_dict = {"x": 400, "y": 250}
    local_dict = {"x": 400, "y": 300}
    cp = [local_dict, local1_dict, ap1_dict, ap_dict]

    # Compute trajectory poins of bezier curve
    pathplan = bezier(cp, 3)

    # Draw scatter
    x = []
    y = []
    for i in range(len(pathplan)):
        x.append(pathplan[i]['x'])
        y.append(pathplan[i]['y'])

    x = np.array(x[:100])
    y = np.array(y[:100])
    f1 = plt.figure(1)
    plt.xlim((300, 450))
    plt.ylim(150, 350)
    plt.scatter(x, y, s=1)

    # Fit cubic curve and obtain coefficient vector
    Para = sp.optimize.curve_fit(curve, x, y)
    a, b, c, d = Para[0]

    xl = np.linspace(350, 400, 1000)
    # Draw curve
    yl = a*xl**3 + b*xl**2 + c*xl**1 + d
    plt.plot(xl, yl, color="red", linewidth=2)

    # Compute and draw tangent line of next point
    k = func_k(x[1], a, b, c)
    kb = y[1] - k * x[1]
    yk = k*xl + kb
    plt.plot(xl, yk, color="black", linewidth=2)

    plt.show()


if __name__ == "__main__":
    main()
