# -- Project information -----------------------------------------------------
project = 'ALICE-LRI'
copyright = '2025 Samuel Soutullo'
author = 'Samuel Soutullo'
release = '0.1.0'

# -- General configuration ---------------------------------------------------
extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.autosummary',
    'sphinx.ext.napoleon',
    'sphinx.ext.viewcode',
    'sphinx.ext.todo',
    'sphinx.ext.intersphinx',
    'sphinx.ext.autosectionlabel',
    'breathe',
    'myst_parser',
    'sphinx_copybutton',
    'sphinx_design',
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

# Furo theme options: show repository link and improve navigation
html_title = "ALICE-LRI Documentation"
html_theme_options = {
    "source_repository": "https://github.com/alice-lri/alice-lri",
    "source_branch": "master",
    "source_directory": "docs/",
    "top_of_page_buttons": ["view", "edit", "github"],
}

# MyST (Markdown) configuration
source_suffix = {
    '.rst': 'restructuredtext',
    '.md': 'markdown',
}
myst_enable_extensions = [
    "colon_fence",
    "deflist",
    "attrs_block",
]

# Intersphinx for cross-project links (optional but handy)
intersphinx_mapping = {
    'python': ('https://docs.python.org/3', None),
}

# Autosectionlabel: prefix labels with document path to avoid collisions
autosectionlabel_prefix_document = True
autosummary_generate = True

# -- Options for todo extension ----------------------------------------------
todo_include_todos = True
