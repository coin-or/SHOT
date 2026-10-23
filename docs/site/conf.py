import os
project = "SHOT"
author = "The SHOT contributors"
release = os.environ.get("SHOT_DOCS_VERSION", "2.0 development")
extensions = ["myst_nb", "sphinx.ext.autodoc", "sphinx.ext.mathjax", "breathe"]
html_theme = "furo"
html_title = "SHOT " + release
html_static_path = ["_static"]
html_css_files = ["branding/tokens.css", "branding/components.css", "shot.css"]
html_js_files = ["shot.js"]
templates_path = ["_templates"]
html_show_sourcelink = False
html_theme_options = {"sidebar_hide_name": False}
breathe_projects = {"SHOT": "../doxygen/xml"}
breathe_default_project = "SHOT"
autodoc_member_order = "bysource"
autodoc_typehints = "description"
nb_execution_mode = "off"
myst_enable_extensions = ["colon_fence", "dollarmath", "amsmath"]
exclude_patterns = ["_assets/**"]
html_extra_path = []
myst_heading_anchors = 4


def format_binding_docstrings(app, what, name, obj, options, lines):
    # Binding prose uses a bare bitwise-OR symbol, which reST reads as substitution markup.
    for index, line in enumerate(lines):
        lines[index] = line.replace("combined with |,", "combined with ``|``,").replace("|x|", "``|x|``").replace("|a - b|", "``|a - b|``")


def setup(app):
    app.connect("autodoc-process-docstring", format_binding_docstrings)
    app.connect("html-page-context", documentation_navigation, priority=900)
html_theme_options.update({
    "light_logo": "shot-logo.png",
    "dark_logo": "shot-logo-dark.png",
})

html_sidebars = {"**": ["sidebar/docs-brand.html", "sidebar/search.html", "sidebar/scroll-start.html", "sidebar/navigation.html", "sidebar/scroll-end.html"]}

# Fira Math matches the sans-serif SHOT typography.
mathjax_path = "https://cdn.jsdelivr.net/npm/mathjax@4.0.0/tex-chtml.js"
mathjax3_config = {"output": {"font": "mathjax-fira"}, "chtml": {"displayAlign": "left"}}


def documentation_navigation(app, pagename, templatename, context, doctree):
    # Furo normally omits page sections; expose them in its accessible dropdown tree.
    from furo.navigation import get_navigation_tree
    if "toctree" in context:
        context["furo_navigation_tree"] = get_navigation_tree(context["toctree"](
            collapse=False, titles_only=False, maxdepth=3, includehidden=True))
    context["furo_hide_toc"] = True
    # Open the active document and its ancestor groups, but keep other pages folded.
    from bs4 import BeautifulSoup
    tree = BeautifulSoup(context.get("furo_navigation_tree", ""), "html.parser")
    current = tree.find("a", href=context["pathto"](pagename))
    if current:
        for item in current.find_parents("li"):
            toggle = item.find("input", class_="toctree-checkbox", recursive=False)
            if toggle:
                toggle["checked"] = ""
    context["furo_navigation_tree"] = str(tree)
