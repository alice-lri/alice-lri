# -- Project information -----------------------------------------------------
project = 'ALICE-LRI'
copyright = '2025 Samuel Soutullo'
author = 'Samuel Soutullo'
release = '0.1.0'

# -- General configuration ---------------------------------------------------
extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.napoleon',
    'sphinx.ext.viewcode',
    'sphinx.ext.todo',
    'breathe',
]

autodoc_typehints = 'description'
napoleon_google_docstring = True
napoleon_numpy_docstring = True

breathe_projects = {"ALICE-LRI": "doxygen/xml"}
breathe_default_project = "ALICE-LRI"

# Exhale configuration
def determine_exhale_args():
    return {
        "containmentFolder": "./api/cpp",
        "rootFileName": "cpp_api.rst",
        "rootFileTitle": "C++ API Reference",
        "doxygenStripFromPath": "../",
        "createTreeView": True,
    }
exhale_args = determine_exhale_args()

html_theme = "furo"
html_static_path = ['_static']

# -- Paths -------------------------------------------------------------------
import os
import sys
sys.path.insert(0, os.path.abspath('../python'))

# -- Options for todo extension ----------------------------------------------
todo_include_todos = True
