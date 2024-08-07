# com2009

Content for the COM2009 ROS lab course site

## Contributing

### Setting up a Python Environment

Site is built using [Material for MKDocs](https://squidfunk.github.io/mkdocs-material/)

First, create a Python virtual environment (ideally 3.9 or higher):

```
python -m venv venv
```

**[Activate the environment](https://realpython.com/what-is-pip/#using-pip-in-a-python-virtual-environment)** and install the latest verison of Material for MKDocs:

```
pip install mkdocs-material
```

The MKDocs [Awesome Pages Plugin](https://github.com/lukasgeiter/mkdocs-awesome-pages-plugin):

```
pip install mkdocs-awesome-pages-plugin
```

And the [Git Revision Date Localised Plugin](https://github.com/timvink/mkdocs-git-revision-date-localized-plugin):

```
pip install mkdocs-git-revision-date-localized-plugin
```

### Editing

Site content is located in the "docs" directory. See here for guidasnce on how to write site content:

* General Guidance on Writing in Markdown: [A Markdown Cheatsheet](https://www.markdownguide.org/cheat-sheet/)
* Material for MkDocs Documentation: https://squidfunk.github.io/mkdocs-material/reference/

To preview the site as you write use the following command (make sure the Python environemnt is active!):

```
mkdocs serve
```

Then go to http://localhost:8000/com2009/ in a browser.