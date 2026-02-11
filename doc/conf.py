# -- Project information -----------------------------------------------------
project = 'Dobot ROS2 Driver Documentation'
copyright = '2026, Your Organization'
author = 'Your Organization'
release = '1.0.0'

# -- General configuration ---------------------------------------------------
extensions = [
    'myst_parser',           # Markdown support
    'sphinx.ext.autodoc',    # Auto-generate from docstrings
    'sphinx.ext.intersphinx',# Cross-reference other Sphinx docs
    'sphinx.ext.todo',       # TODO directives
]

# Markdown support
source_suffix = {
    '.rst': 'restructuredtext',
    '.md': 'markdown',
}

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

# -- Options for HTML output -------------------------------------------------
html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']
html_logo = '_static/logo.png'         # Your logo (optional)
html_theme_options = {
    'logo_only': False,
    'display_version': True,
    'navigation_depth': 3,
    'collapse_navigation': False,
}

# -- Intersphinx mapping (link to ROS2 docs) --------------------------------
intersphinx_mapping = {
    'ros2': ('https://docs.ros.org/en/humble/', None),
}