import CyKalman
import sys
import kalman_adaptor

py_or_cy = sys.argv[1] if len(sys.argv) > 1 else "py"

if py_or_cy != "py" and py_or_cy != "cy":
    raise "1st arg needs to be 'py' or 'cy'"

adaptor = kalman_adaptor.get_kalman_filter(py_or_cy)

k = adaptor.KalmanJerk1D(1.0, 4.0, 1.0)
# https://stackoverflow.com/questions/35112511/pip-setup-py-bdist-wheel-no-longer-builds-forced-non-pure-wheels
a = None
b = None
for i in range(1000000+1):
    #print ("running:" + str(float(i)))
    time = float(i)
    x = (float(i))*2.0
    k.step(time,x)
    a = k.get_kalman_vector()
    b = k.get_eular_vector()

    #print("step:"+str(i)+"|time:" + str(time) + "|x:" + str(x))
    #print ("eular:", k.get_eular_vector())
    #print ("X:", k.get_kalman_vector())
    #print ("P", k.get_covariance_matrix())
    #print("-----------------")
print("kalman")
print(a)
print("eular")
print(b)

"""
t,x 
1,0
2,2
3,4


"""
    