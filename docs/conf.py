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
# Don't add module names to documented objects (cleaner display)
add_module_names = False
# Use simpler type representations
autodoc_typehints_format = 'short'
# Custom processing to simplify type hints
def process_signature(app, what, name, obj, options, signature, return_annotation):
    if signature:
        # Replace verbose type hints with simpler ones
        signature = signature.replace('alice_lri._alice_lri.', 'alice_lri.')
        signature = signature.replace('collections.abc.Sequence[typing.SupportsFloat]', 'Sequence[float]')
        signature = signature.replace('typing.SupportsFloat', 'float')
        signature = signature.replace('typing.SupportsInt', 'int')
    if return_annotation:
        return_annotation = return_annotation.replace('alice_lri._alice_lri.', 'alice_lri.')
        return_annotation = return_annotation.replace('collections.abc.Sequence[typing.SupportsFloat]', 'Sequence[float]')
        return_annotation = return_annotation.replace('typing.SupportsFloat', 'float')
        return_annotation = return_annotation.replace('typing.SupportsInt', 'int')
    return (signature, return_annotation)

def setup(app):
    app.connect('autodoc-process-signature', process_signature)

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
html_title = "Documentation"
html_theme_options = {
    "light_logo": "logo.svg",
    "dark_logo": "logo-dark.svg",
    "source_repository": "https://github.com/alice-lri/alice-lri",
    "source_branch": "master",
    "source_directory": "docs/",
    "top_of_page_buttons": ["view", "edit"],
    "footer_icons": [
        {
            "name": "GitHub",
            "url": "https://github.com/alice-lri/alice-lri",
            "html": """
                <svg stroke="currentColor" fill="currentColor" stroke-width="0" viewBox="0 0 16 16">
                    <path fill-rule="evenodd" d="M8 0C3.58 0 0 3.58 0 8c0 3.54 2.29 6.53 5.47 7.59.4.07.55-.17.55-.38 0-.19-.01-.82-.01-1.49-2.01.37-2.53-.49-2.69-.94-.09-.23-.48-.94-.82-1.13-.28-.15-.68-.52-.01-.53.63-.01 1.08.58 1.23.82.72 1.21 1.87.87 2.33.66.07-.52.28-.87.51-1.07-1.78-.2-3.64-.89-3.64-3.95 0-.87.31-1.59.82-2.15-.08-.2-.36-1.02.08-2.12 0 0 .67-.21 2.2.82.64-.18 1.32-.27 2-.27.68 0 1.36.09 2 .27 1.53-1.04 2.2-.82 2.2-.82.44 1.1.16 1.92.08 2.12.51.56.82 1.27.82 2.15 0 3.07-1.87 3.75-3.65 3.95.29.25.54.73.54 1.48 0 1.07-.01 1.93-.01 2.2 0 .21.15.46.55.38A8.013 8.013 0 0 0 16 8c0-4.42-3.58-8-8-8z"></path>
                </svg>
            """,
            "class": "",
        },
    ],
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
