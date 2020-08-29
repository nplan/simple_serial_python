import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="simple_serial_python",
    version="2.0.0",
    author="Nejc Planinsek",
    author_email="planinseknejc@gmail.com",
    description="A serial communication package",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/pnplan/simple_serial_python",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    install_requires=[
        "pyserial",
    ],
)
