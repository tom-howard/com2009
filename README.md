# The COM2009 Course Site

Course material for the COM2009 ROS2 Course Assignments.

Access the site here: https://tom-howard.github.io/com2009/

## Contributing

### Setting up a Python Environment

Site is built using [Material for MKDocs](https://squidfunk.github.io/mkdocs-material/)

First, create a Python virtual environment (ideally 3.9 or higher):

```
python -m venv venv
```

**[Activate the environment](https://realpython.com/what-is-pip/#using-pip-in-a-python-virtual-environment)** and install the latest version of Material for MKDocs along with the following plugins:

* MKDocs [Awesome Pages Plugin](https://github.com/lukasgeiter/mkdocs-awesome-pages-plugin)
* [Git Revision Date Localised Plugin](https://github.com/timvink/mkdocs-git-revision-date-localized-plugin)
* [Plugins for social card generation](https://squidfunk.github.io/mkdocs-material/plugins/requirements/image-processing/)

```
pip install \
    mkdocs-material \
    mkdocs-awesome-pages-plugin \
    mkdocs-git-revision-date-localized-plugin \
    "mkdocs-material[imaging]"
```

### Editing

Site content is located in the "docs" directory. See here for guidance on how to write site content:

* General Guidance on Writing in Markdown: [A Markdown Cheatsheet](https://www.markdownguide.org/cheat-sheet/)
* Material for MkDocs Documentation: https://squidfunk.github.io/mkdocs-material/reference/

To preview the site as you write use the following command (make sure the Python environment is active!):

```
mkdocs serve
```

Then go to http://localhost:8000/com2009/ in a browser.