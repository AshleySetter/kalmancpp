from distutils.core import setup

with open('requirements.txt') as f:
    requirements = f.read().splitlines()

setup(name='kalmancpp',
      version='0.0.1',
      description='Python package providing linear Kalman Filter in Cpp with Cython',
      author='Ashley Setter',
      auther_email='A.Setter@soton.ac.uk',
      url=None,
      packages=['kalmancpp'],
      package_dir={'kalmancpp': 'kalmancpp',
      },
      install_requires=requirements,
)
