# Spellcheck

The template uses [sphinxcontrib.spelling](https://sphinxcontrib-spelling.readthedocs.io) plugin for spellcheck.

## Usage

After you have edited your text files you can issue `make spelling` to check for spelling errors. The output will list you the spelling errors along with some suggestions.

Typically some technical words are not going to pass the spellcheck. You have to append these words to the file configured in `conf.py`:

```{literalinclude} conf.py
:start-at: spelling_word_list
:end-at: spelling_word_list
```

For example the spelling word list in this project contains:

```{literalinclude} spelling_word_list.txt
```

## Continuous integration

The continuous integration (CI) has automatic spellcheck integrated, so if you have spelling errors after you pushed your code to the repository, then the spellcheck stage of the CI pipeline will fail.

```{note}
The spellcheck activates `-W` option in Sphinx, which treats warnings as errors. If you already get some warnings before spellcheck, this may lead to compilation errors and thus break your pipeline.
```

If you want to deactivate spellcheck in CI, then remove the following lines from `.gitlab-ci.yml`:

```{literalinclude} .gitlab-ci.yml
:start-at: spellcheck
:end-before: deploy-webpages
```
