# FsAnalyzer
A work in progress tool for plotting bicycle suspension antisquat and leverage
rate curves

![](data/antisquat_figure.png)


## Installation
This step installs the required `matplotlib`, `numpy` and `pyyaml` dependencies
if they are not already available on your machine. It also installs the 
`fsanalyzer` package to your python path.
```
git clone https://github.com/shift-dynamics/fsanalyzer
cd fsanalyzer
pip install . --user
```

## Usage
### Run the test program
From the main repo directory, run the test program as:
```
python test/test_draw_antisquat.py
```

### Use in your python code
To use the package in your python code:
```python
#!/usr/bin/env python

from fsanalyzer.kinematics import FsAnalyzer

if __name__ == "__main__":
    fsanalyzer = FsAnalyzer("config.yaml")
    fsanalyzer.draw()
```
where `config.yaml` is the path to a yaml configuration file, see 
`data/example.yaml` for an example of the syntax.
