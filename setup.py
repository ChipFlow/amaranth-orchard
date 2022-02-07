from setuptools import setup, find_packages

def scm_version():
    def local_scheme(version):
        if version.tag and not version.distance:
            return version.format_with("")
        else:
            return version.format_choice("+{node}", "+{node}.dirty")
    return {
        "relative_to": __file__,
        "version_scheme": "guess-next-dev",
        "local_scheme": local_scheme
    }

with open("README.md", "r") as f:
    long_description = f.read()

setup(
    name="amaranth-orchard",
    use_scm_version=scm_version(),
    author="Myrtle Shah",
    author_email="gatecat@ds0.me",
    description="curated collection of Amaranth cores",
    long_description=long_description,
    long_description_content_type="text/markdown",
    python_requires=">=3.6",
    setup_requires=["setuptools_scm"],
    include_package_data=False,
    packages=find_packages(),
    package_data={
    	'amaranth_orchard': ['memory/verilog/*.v']
    },
    project_urls={
        "Source Code": "https://gitlab.com/ChipFlow/amaranth-orchard",
        "Bug Tracker": "https://gitlab.com/ChipFlow/amaranth-orchard/issues",
    },
)
