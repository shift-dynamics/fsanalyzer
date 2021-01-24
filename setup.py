from setuptools import setup, find_packages


if __name__ == "__main__":
    setup(
        name="fsanalyzer",
        version="0.0.1",
        description="A tool to visualize bicycle suspension kinematics",
        author="Kristopher Wehage",
        packages=find_packages(include=["fsanalyzer"]),
        install_requires=[
            "numpy>=1.14.5",
            "matplotlib>=2.2.0",
        ]
    )
