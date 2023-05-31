from setuptools import setup, Extension
from Cython.Build import cythonize
import numpy

ext = Extension("CyKalman",
                sources=["lib/jerk/KalmanJerk1D.pyx"], #"kalman.cpp"
                language="c++",
                include_dirs=[numpy.get_include()]
               )

setup(
    name='CyKalman',
    version='1.0',
    ext_modules=cythonize(ext),
    zip_safe=False,
)


"""
setup(
    name='Kalman1D',
    version='1.0',
    ext_modules=cythonize("Kalman1D.pyx"),
    zip_safe=False,
)

ext = Extension("myproj",
                sources=["myproj.pyx", "myCppProjFacade.cpp"],
                <etc>
                language="c++"
               )


from setuptools import setup
from Cython.Build import cythonize

setup(
    name='Kalman1D',
    ext_modules=cythonize("kalman.pyx"),
    zip_safe=False,
)


from Cython.Build import cythonize
from setuptools import setup, Extension

setup(  name='Kalman1D',
        version='1.0',
        py_modules=['Kalman'],
        ext_modules =
            cythonize(Extension("kalman",
                        # the extension name
                sources=["kalman.pyx", "kalman.cpp"],
                        # the Cython source and additional C++ source files
                language="c++", # generate and compile C++ code
                extra_compile_args=['-g']
                                )
                        )
    )

"""