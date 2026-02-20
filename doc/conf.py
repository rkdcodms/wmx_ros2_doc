# -- Project information -----------------------------------------------------
project = 'WMX ROS2 Documentation'
copyright = '2026, Movensys'
author = 'Movensys'
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
html_theme = 'pydata_sphinx_theme'
html_static_path = ['_static']
html_css_files = ['custom.css']

html_logo = '_static/movensys_logo.png'
html_favicon = '_static/movensys_logo.png'

html_show_sourcelink = False
html_last_updated_fmt = '%b %d, %Y'

html_theme_options = {
    "logo": {
        "text": "WMX ROS2 Documentation",
    },
    "icon_links": [
        {
            "name": "GitHub",
            "url": "https://github.com/movensys/wmx_ros2_doc",
            "icon": "fa-brands fa-github",
            "type": "fontawesome",
        },
    ],
    "navbar_align": "left",
    "navigation_depth": 4,
    "show_nav_level": 1,
    "show_toc_level": 2,
    "collapse_navigation": True,
    "navbar_start": ["navbar-logo"],
    "navbar_center": [],
    "navbar_end": ["search-button", "navbar-icon-links"],
    "primary_sidebar_end": [],
    "sidebar_includehidden": True,
    "footer_start": [],
    "footer_end": [],
    "secondary_sidebar_items": ["page-toc"],
}

html_sidebars = {
    "**": ["sidebar-toc-header", "sidebar-nav-global"],
}

html_context = {
    "default_mode": "light",
}

# -- Intersphinx mapping (link to ROS2 docs) --------------------------------
intersphinx_mapping = {
    'ros2': ('https://docs.ros.org/en/humble/', None),
}
