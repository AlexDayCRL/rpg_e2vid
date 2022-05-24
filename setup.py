from setuptools import setup

setup(name='rpg_e2vid',
      version='0.0.1',
      description='Package release of the E2VID project',
      url='http://github.com/AlexDayCRL/rpg_e2vid',
      author='Alex Day',
      author_email='aday@carnegierobotics.com',
      license='GPLv3',
      packages=["rpg_e2vid.base", "rpg_e2vid.model", "rpg_e2vid.utils"],
      package_dir = {
          "rpg_e2vid.base": "./base",
          "rpg_e2vid.model": "./model",
          "rpg_e2vid.utils": "./utils"
      },
      zip_safe=False
)
