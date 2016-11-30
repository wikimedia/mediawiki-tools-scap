#!/usr/bin/env python2

import os.path

from distutils.core import setup

authors = [('Antoine Musso', 'hashar@free.fr'),
           ('Bryan Davis', 'bd808@wikimedia.org'),
           ('Chad Horohoe', 'chadh@wikimedia.org'),
           ('Dan Duvall', 'dduvall@wikimedia.org'),
           ('Mukunda Modell', 'mmodell@wikimedia.org'),
           ('Ori Livneh', 'ori@wikimedia.org'),
           ('Tyler Cipriani', 'tcipriani@wikimedia.org')]


# Read version from file shared with the module using technique from
# https://python-packaging-user-guide.readthedocs.io/en/latest/single_source_version/
base_dir = os.path.dirname(__file__)
version = {}
execfile(os.path.join(base_dir, 'scap', 'version.py'), version)

setup(name='Scap',
      version=version['__version__'],
      description='Deployment toolchain for Wikimedia projects',
      author=', '.join([name for name, _ in authors]),
      author_email=', '.join([email for _, email in authors]),
      license='GNU GPLv3',
      maintainer='Wikimedia Foundation Release Engineering',
      maintainer_email='releng@wikimedia.org',
      url='https://phabricator.wikimedia.org/diffusion/MSCA/',
      packages=['scap', 'scap.plugins'],
      package_dir={'scap': 'scap'},
      scripts=['bin/scap'],
      requires=[line.strip() for line in open('requirements.txt')],
      classifiers=['Operating System :: POSIX :: Linux',
                   'Programming Language :: Python',
                   'Programming Language :: Python :: 2',
                   'Programming Language :: Python :: 2.7'])
