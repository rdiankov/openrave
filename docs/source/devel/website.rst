Managing openrave.org
---------------------

openrave.org uses Django for managing documentation, news, and blogs. The code is maintained at::

   https://github.com/rdiankov/openrave.org

Using system similar to `djangoproject.com <https://github.com/django/djangoproject.com>`_

Install
=======

1. Create a virtualenv
2. Install dependencies:
  
  .. code-block:: bash
  
    apt-get install postgresql postgresql-client libpq-dev memcached python-dev gettext
    pip install -r deploy-requirements.txt
  
  If you only need to deploy, and don't need to test any changes, you can use local-requirements.txt
  
  Dependencies for Apache Webserver Deployment:
  
  .. code-block:: bash
  
    apt-get install libapache2-mod-wsgi
    a2enmod wsgi

3. Set up databases, as per django_website/settings/www.py

4. Create a 'openrave.org_secrets.json' file in the directoy above the checkout, containing
   something like::

    { "secret_key": "xyz",
      "superfeedr_creds": ["any@email.com", "some_string"] }

5. Initial DB Setup::
  
    ./manage.py syncdb
    ./manage.py convert_to_south docs
  
  Future DB Update::
  
    ./manage.py syncdb
    ./manage.py migrate

6. For Docs::
  
    ./manage.py loaddata doc_releases.json
    ./manage.py update_docs 

7. For adding new document:
  
  .. code-block:: bash
  
    export OPENRAVE_VERSION=0.8.0
    export DOC_LANG=en
    DJANGO_SETTINGS_MODULE=openrave_website.settings python -c "from openrave_website.docs import models; models.DocumentRelease.objects.create(lang='$DOC_LANG',version='$OPENRAVE_VERSION', scm=models.DocumentRelease.GIT, scm_url='https://github.com/rdiankov/openrave/tree/v$OPENRAVE_VERSION', is_default=False);"
  
  Re-index the documents:
  
  .. code-block:: bash
  
    ./manage.py update_docs
  
  Mini script to convert latest_stable docdata to a specific version:

  .. code-block:: bash
  
    export OPENRAVE_VERSION=0.8.0
    unzip openravejson-latest_stable.zip
    mv openravejson-latest_stable openravejson-$OPENRAVE_VERSION
    zip -r openravejson-$OPENRAVE_VERSION.zip openravejson-$OPENRAVE_VERSION
    unzip openravehtml-latest_stable.zip
    mv openravehtml-latest_stable openravehtml-$OPENRAVE_VERSION
    zip -r openravehtml-$OPENRAVE_VERSION.zip openravehtml-$OPENRAVE_VERSION

8. Internationalization. For Japanese, edit **locale/ja_JP/LC_MESSAGES/django.po** file::

    django-admin.py makemessages --locale=ja_JP
    django-admin.py compilemessages --locale=ja_JP

9. Running Locally::

    python manage.py runserver

10. For deployment checkout fabfile.py::

    https://openrave.svn.sourceforge.net/svnroot/openrave/openrave.org/fabfile.py

Creating PostgreSQL Database
============================

.. code-block:: bash

  sudo -u postgres psql --command "CREATE ROLE openrave PASSWORD 'testpass' SUPERUSER CREATEDB CREATEROLE INHERIT LOGIN;"
  createdb --host localhost --username openrave --encoding UTF-8 openrave_website

Editing
=======

All HTML templates are in **openrave_website/templates**. `Tutorial on HTML + Django syntax template language <https://docs.djangoproject.com/en/1.4/topics/templates/>`_

Only write English in the HTML files and only inside these translation blocks:

- `trans <https://docs.djangoproject.com/en/1.4/topics/i18n/translation/#std:templatetag-trans>`_

- `blocktrans <https://docs.djangoproject.com/en/1.4/topics/i18n/translation/#blocktrans-template-tag>`_  

Videos and image filenames should also be written within the translation blocks so that we can substitute them with the language equivalent.

Translating to Japanese
=======================

When English templates are done, execute:

.. code-block:: bash

  django-admin.py makemessages --locale=ja_JP

Open **locale/ja_JP/LC_MESSAGES/django.po** and edit the translations. When done execute:

.. code-block:: bash

  django-admin.py compilemessages --locale=ja_JP

Restart the mujinwww server and the new translation should be visible!

Maintain
========

`GetSentry Account <https://app.getsentry.com/openrave/group/182445/>`_
