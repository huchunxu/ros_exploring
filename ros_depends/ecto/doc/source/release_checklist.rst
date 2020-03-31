Release Checklist
=================

* Be sure you are fully merged::
  
    git submodule foreach git fetch --all
    git submodule foreach git pull plasmodic master

* Verify testing.

* Pick your tag.  We'll call this one ``amoeba-0``

* Tweak version information in ecto/cmake/version.hpp

* If desired update ABI info in ecto/include/ecto/abi.hpp

* Add to list in ecto/doc/source/changelog_gen.py

* make sure you're synced::
   
    git submodule foreach git fetch --all

* check yer diffs::

    git submodule foreach git diff --stat plasmodic/master

* verify that your pushes will succeed::

    git push --dry-run plasmodic master
    git submodule foreach git push --dry-run plasmodic master

* Tag kitchen and projects with the same tag::

    % git submodule foreach git tag amoeba-0
    Entering 'ecto'
    Entering 'opencv'
    Entering 'openni'
    Entering 'pcl'
    Entering 'ros'
    % git tag amoeba-0

* Make docs, verify they look good.
    
* if okay, push::

    git submodule foreach git push plasmodic master

* and push tags::

    git submodule foreach git push --tags plasmodic

* and do the kitchen::

    git push plasmodic master
    git push --tags plasmodic

* make the docs and deploy to the website
* Change tag/link on main ecto page to latest release (commit to www proj)



