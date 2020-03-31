Coding Standards
================

This is something one doesn't want to overlegislate.

We mainly follow the coding standards of `Sutter and Alexandrescu
<http://www.amazon.com/Coding-Standards-Rules-Guidelines-Practices/dp/0321113586>`_.

A few specific guidelines:

* **NO TABS** in your commits.
* **NO TRAILING WHITESPACE** in the code.

The two above keep noise out of our commit history.  Things we strive for:

* Put the bugfix, the test of the bugfix, and the updated docs in the
  *same commit*, via ``git rebase`` if necessary.  We're not kernel
  ninjas, but we strive to be as good with source control as they
  are.

git pre-commit hooks
--------------------

To help with trailing white space, the user may want to enable the
default git pre-commit hook.

.. highlight:: ectosh

::

  % cd my_git_repo
  % cp .git/hooks/pre-commit.sample .git/hooks/pre-commit

This will cause git commit to error out before you are even allowed to enter
a commit message if obvious snaffoos are in your diff.

::

  % git commit -a
   kitchen/doc/source/codingstandards.rst:32: trailing whitespace.
   +a commit message if obvious snaffoos are in your diff.

See http://book.git-scm.com/5_git_hooks.html for more detailed information on using
git to help with the quality of your commits.

