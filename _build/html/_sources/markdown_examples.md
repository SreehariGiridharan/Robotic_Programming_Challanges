# Markdown examples

Emphasis using *Italics*, **bold**.
The paragraph continues

This is the second paragraph.

By increasing the number of `#` symbols, we can create subchapters, sub-subchapters, etc.

## Block quotes

Kermit from the Muppet Movie:

> Life's like a movie. Write your own ending.

## Lists

- apple
- banana
- cherry

## Links

More info can be found on [Python docs – Exceptions](https://docs.python.org/3/reference/executionmodel.html#exceptions).

## Images

![Charles J. Sharp, CC BY-SA 4.0 <https://creativecommons.org/licenses/by-sa/4.0>, via Wikimedia Commons](https://upload.wikimedia.org/wikipedia/commons/thumb/8/81/Scarlet_darter_%28Crocothemis_erythraea%29_female_Bulgaria.jpg/320px-Scarlet_darter_%28Crocothemis_erythraea%29_female_Bulgaria.jpg)

## Code snippets

We can use `find -iname STRING` to search for a file in Unix.

The following code[^1] shuffles a list:

```python
from random import shuffle

foo = [1, 2, 3, 4]
shuffle(foo) 
print(foo)
```

## The end

If you want to try these, follow this [Markdown tutorial](https://commonmark.org/help/tutorial). It also includes a web-based code editor.

[^1]: Code from [PythonSnippets.dev](https://pythonsnippets.dev/snippet/23/)
